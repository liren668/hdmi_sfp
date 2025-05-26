//----------------------------------------------------------------------------------------
// File name:           sfp_tx_module.v
// Created by:          ɺ����˹��
// Created date:        2025.5
// Version:             V0.1
// Descriptions:        SFP����ģ�飬�������ζ��кͷ����߼�
//                      HDMI����148.5MHzд�뻷�ζ��У�156.25MHz�������͵�SFP
//----------------------------------------------------------------------------------------

module sfp_tx_rx_module(
    // ϵͳ�ź�
    input                    sys_rst_n,           // ϵͳ��λ�źţ��͵�ƽ��Ч
    input                    locked_0,            // ���໷�����ź�
    input                    wr_clk,              // дʱ�� 148.5MHz
    input                    sys_clk,             // ϵͳʱ��100MHz
    // input                    clk_ila,             // ILAʱ��300MHz

    // HDMI��������ӿڣ�148.5MHzʱ����
    input                 hdmi_data_valid,     // HDMI������Ч
    input      [63:0]     hdmi_data,           // HDMI����

    // HDMI��������ӿ�(�ַ���HDMIģ��)
    output reg      hdmi_axi_rx_valid,      // HDMI����������Ч
    output reg [63:0] hdmi_axi_rx_data,      // HDMI��������

    // SFP����ӿڶ˿ڲ�ȫ
    input           q0_ck1_n_in,            // �ο�ʱ�Ӹ���156.25MHz
    input           q0_ck1_p_in,            // �ο�ʱ������156.25MHz
    input           rxn_in,                 // SFP��ֽ��ո���
    input           rxp_in,                 // SFP��ֽ�������
    output          txn_out,                // SFP��ַ��͸���
    output          txp_out,                // SFP��ַ�������
    output          tx_disable,             // SFP����ʹ��
    output          tx_clk_out              // SFP����ʱ��156.25MHz
);

//==========================================================================
// ��������
//==========================================================================
parameter LOCAL_NODE_INFO = 8'h12;      // ���ؽڵ���Ϣ
localparam BOUNDARY_DATA = {48'hFEFEFEFEFEFE, 2'b11, 6'd0, LOCAL_NODE_INFO};
// ���ζ��в���
localparam RING_DEPTH = 64;              // 64����ȵĻ��ζ���
localparam RING_ADDR_WIDTH = 6;          // ���ζ��е�ַ���Ϊ6λ
// �������Ͷ���
parameter DATA_TYPE_HDMI = 4'h2;    // HDMI�������� 0010
parameter DATA_TYPE_ETH = 4'h1;     // ��̫���������� 0001

//==========================================================================
// hdmi�ڲ��źŶ���
//==========================================================================
// ���ζ��д洢
reg [63:0]      hdmi_ring_data[0:RING_DEPTH-1];  // ���ζ������ݣ�64λ��
reg [RING_ADDR_WIDTH-1:0] hdmi_wr_ptr;           // дָ�루148.5MHzʱ����
reg [RING_ADDR_WIDTH-1:0] hdmi_rd_ptr;           // ��ָ�루156.25MHzʱ����

// ��ʱ����ͬ���ź�
(* ASYNC_REG = "TRUE" *) reg [RING_ADDR_WIDTH-1:0] hdmi_sync_wr_ptr_1;  // дָ��ͬ����һ��
(* ASYNC_REG = "TRUE" *) reg [RING_ADDR_WIDTH-1:0] hdmi_sync_wr_ptr;    // дָ��ͬ���ڶ���

// ����״̬�ź�
wire            hdmi_ring_empty;         // ���ζ��пձ�־
wire            hdmi_ring_full;          // ���ζ�������־������дʱ����
reg [RING_ADDR_WIDTH-1:0] hdmi_next_wr_ptr;   // ��һ��дָ��

// ���Ϳ����ź�
reg  [3:0]      send_type_cnt;           // �������ͼ�����

// �߽�������ʱ��������ź�
reg  [3:0]      hdmi_boundary_delay_cnt;      // HDMI�߽�������ʱ��������0-7��8�����ڣ�
reg             hdmi_boundary_detected;       // HDMI�߽����ݼ���־
wire            hdmi_clear_all;               // ��������HDMI���ݺ�ָ����ź�

// hdmi_clear_all��ʱ����ͬ������ʱ���� �� дʱ����
(* ASYNC_REG = "TRUE" *) reg hdmi_clear_sync1;     // �����ź�ͬ����һ��
(* ASYNC_REG = "TRUE" *) reg hdmi_clear_sync2;     // �����ź�ͬ���ڶ���

//==========================================================================
// �շ���IP���ڲ��źŶ���
//==========================================================================
//SFP reg define
reg  [27:0] dely_500ms ;
reg  [2:0]  S_RESET;            //״̬�ź�
reg  [15:0] cnt_rst;
reg         rst_n;
reg         gtwiz_reset_rx_datapath;
//SFP wire define
wire        stat_rx_status;
wire        tx_reset;
wire        rx_reset;
wire        gt0_rst;
wire        gtpowergood_out_0;        // GT��Դ״̬�ź�
wire        tx_clk_out;             // ����ʱ��
wire        gt_refclk_out;          // �ο�ʱ��
wire        axis_to_sfp_tready;     // ����ʹ��
wire [55:0] rx_preambleout;  // ����ǰ����
wire        rx_axis_tuser;
wire        tx_axis_tuser;

// AXI���ͽӿڣ����͵�SFP��
reg         axis_to_sfp_tvalid;     // ������Ч�ź�
reg  [63:0] axis_to_sfp_tdata;      // ��������
reg         axis_to_sfp_tlast;      // ���һ�ı�־
reg  [7:0]  axis_to_sfp_tkeep;      // �ֽ���Ч��־
// AXI���սӿڣ���SFP���գ�
wire        sfp_to_axis_tvalid;     // ������Ч
wire [63:0] sfp_to_axis_tdata;      // �������ݴ�SFP 
wire        sfp_to_axis_tlast;      // ���һ��
wire [7:0]  sfp_to_axis_tkeep;      // �ֽ���Ч
// ��λ����
always@(posedge sys_clk)begin
    if(!locked_0)
        cnt_rst <= 16'b0;
    else if(!cnt_rst[15])
        cnt_rst <= cnt_rst + 1'b1;
    else
        cnt_rst <= cnt_rst;
end 

//SFP�շ�����λ�źţ��͵�ƽ��Ч
assign gt0_rst = ~rst_n;    // ϵͳ��λʱGT��λ���ߵ�ƽ��λ
always@(posedge sys_clk)begin
    if(!locked_0)
        rst_n <= 1'b0;
    else if(cnt_rst > 16'd10000 && cnt_rst <= 16'd20000)
        rst_n <= 1'b0;
    else if(cnt_rst > 16'd20000 && cnt_rst <= 16'd30000)
        rst_n <= 1'b1;
    else
        rst_n <= rst_n;
end

//500ms������
always @(posedge sys_clk)begin
    if(!rst_n)begin
        S_RESET <= 1'b0;
        gtwiz_reset_rx_datapath <= 1'b0;
    end
    else begin
        case(S_RESET)
        0 : begin
            if(dely_500ms == 800_000_00)begin
                dely_500ms  <= 0;
                S_RESET <= 1;
            end
            else
                dely_500ms <= dely_500ms + 1'b1;
        end
        1 : begin
            gtwiz_reset_rx_datapath <= 1'b1;
            if(!(stat_rx_status))
                S_RESET <= 2;
            else 
                S_RESET <= 3;
        end
        2 : begin
            gtwiz_reset_rx_datapath <= 1'b0;
            S_RESET <= 0;          
        end
        3 : begin
            gtwiz_reset_rx_datapath <= 1'b0;
            S_RESET <= 4;            
        end
        4 : begin
            if(dely_500ms == 800_000_00)begin
                dely_500ms <= 0;
                S_RESET <= 5;
            end
            else
                dely_500ms <= dely_500ms + 1'b1;
        end
        5 : begin
            if(!(stat_rx_status))
                S_RESET <= 1;
            else 
                S_RESET <= S_RESET;
        end
        endcase  
    end
end

//ILAʵ����
// ila_1 u_ila_1 (
//     .clk(clk_ila),              // input wire clk300Mhz
//     .probe0(hdmi_data_valid),   // input wire [63:0]  probe0  
//     .probe1(hdmi_data),  // input wire [7:0]  probe1 
//     .probe2(axis_to_sfp_tvalid),   // input wire [0:0]  probe2 
//     .probe3(axis_to_sfp_tlast),  // input wire [7:0]  probe6 
//     .probe4(axis_to_sfp_tdata),  // input wire [7:0]  probe6 
//     .probe5(hdmi_axi_rx_valid),   // input wire [23:0]  probe4 
//     .probe6(hdmi_axi_rx_data),   // input wire [7:0]  probe6 
//     .probe7(sfp_to_axis_tvalid),  // input wire [7:0]  probe6 
//     .probe8(sfp_to_axis_tdata),   // input wire [7:0]  probe6 
//     .probe9(sfp_to_axis_tlast)   // input wire [7:0]  probe6 
//     // .probe10()   // input wire [7:0]  probe6 
//     // .probe12(video_de_posedge),   // video_de�����ؼ��
//     // .probe13(video_de_negedge)    // video_de�½��ؼ��
//     // .probe12(test_valid),   // input wire [7:0]  probe6 
//     // .probe13(test_data)    // input wire [0:0]  probe5 
// );

//==========================================================================
// ���ζ���״̬�ж�
//==========================================================================
// ������һ��дָ��
always @(*) begin
    hdmi_next_wr_ptr = (hdmi_wr_ptr == RING_DEPTH-1) ? {RING_ADDR_WIDTH{1'b0}} : hdmi_wr_ptr + 1'b1;
end

// ���ζ������жϣ�дʱ����
assign hdmi_ring_full = (hdmi_next_wr_ptr == hdmi_rd_ptr);

// ���ζ��п��жϣ���ʱ����
assign hdmi_ring_empty = (hdmi_rd_ptr == hdmi_sync_wr_ptr);

//==========================================================================
// ���ζ���д���߼���148.5MHzʱ����
// ֻ������wr_clkʱ�����hdmi_ring_data��ֵ�����㣬�������������
//==========================================================================
integer i;  // ����ѭ����ʼ������������

always @(posedge wr_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        hdmi_wr_ptr <= {RING_ADDR_WIDTH{1'b0}};
        hdmi_clear_sync1 <= 1'b0;
        hdmi_clear_sync2 <= 1'b0;
        // ��ʼ�����ζ���
        for(i = 0; i < RING_DEPTH; i = i + 1) begin
            hdmi_ring_data[i] <= 64'd0;
        end
    end else begin
        // ͬ��hdmi_clear_all�źŵ�дʱ����
        hdmi_clear_sync1 <= hdmi_clear_all;
        hdmi_clear_sync2 <= hdmi_clear_sync1;
        // ��⵽�����ź�ʱ����дָ��ͻ��ζ���
        if (hdmi_clear_sync2) begin
            hdmi_wr_ptr <= {RING_ADDR_WIDTH{1'b0}};
            for(i = 0; i < RING_DEPTH; i = i + 1) begin
                hdmi_ring_data[i] <= 64'd0;
            end
        end else begin
            // ������Ч�����Ҷ��в���ʱ��д������
            if (hdmi_data_valid && !hdmi_ring_full) begin
                hdmi_ring_data[hdmi_wr_ptr] <= hdmi_data;           // д��64λHDMI����
                hdmi_wr_ptr <= hdmi_next_wr_ptr;                    // �ƶ�дָ��
            end
        end
    end
end

//==========================================================================
// ��ʱ����ͬ����156.25MHzʱ����
//==========================================================================
always @(posedge tx_clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        hdmi_sync_wr_ptr_1 <= {RING_ADDR_WIDTH{1'b0}};
        hdmi_sync_wr_ptr <= {RING_ADDR_WIDTH{1'b0}};
    end else begin
        hdmi_sync_wr_ptr_1 <= hdmi_wr_ptr;                     // ��һ��ͬ��
        hdmi_sync_wr_ptr <= hdmi_sync_wr_ptr_1;                // �ڶ���ͬ��
    end
end

//==========================================================================
// SFP�����߼���156.25MHzʱ����
// ע�⣺�˴����ٶ�hdmi_ring_data��������κθ�ֵ�����������
//==========================================================================
always @(posedge tx_clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        axis_to_sfp_tvalid <= 1'b0;
        axis_to_sfp_tdata  <= 64'h0;
        axis_to_sfp_tlast  <= 1'b0;
        axis_to_sfp_tkeep  <= 8'h0;
        hdmi_rd_ptr        <= {RING_ADDR_WIDTH{1'b0}};
        send_type_cnt      <= 4'd0;
        hdmi_boundary_detected <= 1'b0;
        hdmi_boundary_delay_cnt <= 4'd0;
    end else begin
        // Ĭ�����
        axis_to_sfp_tvalid <= 1'b0;
        axis_to_sfp_tdata  <= 64'h0;
        axis_to_sfp_tlast  <= 1'b0;
        axis_to_sfp_tkeep  <= 8'h0;

        // �߽�������ʱ�����߼�
        if (hdmi_clear_all) begin
            // ֻ�����ָ�����ر�־����������hdmi_ring_data����
            hdmi_rd_ptr <= {RING_ADDR_WIDTH{1'b0}};
            hdmi_boundary_detected <= 1'b0;      // �������־
            hdmi_boundary_delay_cnt <= 4'd0;     // ���������
        end else if (hdmi_boundary_detected) begin
            // �߽����ݼ���ʼ��ʱ����
            hdmi_boundary_delay_cnt <= hdmi_boundary_delay_cnt + 1'b1;
        end

        // ����LOCAL_NODE_INFO��4λ��1��λ����ѯ����
        case (send_type_cnt)
            4'd0: begin  // ��0�����ݣ�HDMI����
                if (LOCAL_NODE_INFO[1]) begin  // ��1λ��ӦHDMI����
                    if (!hdmi_ring_empty && axis_to_sfp_tready) begin
                        // ֱ��������ζ�������
                        axis_to_sfp_tvalid <= 1'b1;
                        axis_to_sfp_tdata  <= hdmi_ring_data[hdmi_rd_ptr];
                        axis_to_sfp_tkeep  <= 8'hFF;
                        // ����������������tlast
                        if ((hdmi_ring_data[hdmi_rd_ptr][15:14] == 2'd3) || (hdmi_ring_data[hdmi_rd_ptr][15:14] == 2'd1))
                            axis_to_sfp_tlast <= 1'b1;
                        else
                            axis_to_sfp_tlast <= 1'b0;
                        
                        // ����Ƿ�Ϊ�߽�����
                        if (hdmi_ring_data[hdmi_rd_ptr] == BOUNDARY_DATA) begin
                            hdmi_boundary_detected <= 1'b1;  // ���ñ߽����־
                            hdmi_boundary_delay_cnt <= 4'd0; // ��ʼ����
                        end
                        
                        // ������ƶ�ָ��
                        hdmi_rd_ptr <= (hdmi_rd_ptr == RING_DEPTH-1) ? {RING_ADDR_WIDTH{1'b0}} : hdmi_rd_ptr + 1'b1;
                    end else begin
                        // ���ζ��п�ʱ��tvalidΪ1�����ȫ0����
                        axis_to_sfp_tvalid <= 1'b1;
                        axis_to_sfp_tdata  <= 64'h0;
                        axis_to_sfp_tlast  <= 1'b0;
                        axis_to_sfp_tkeep  <= 8'h00;
                    end
                end
                
                // ״̬ת��������LOCAL_NODE_INFO������һ��״̬
                if (LOCAL_NODE_INFO[0]) begin
                    send_type_cnt <= 4'd1;  // ��һ���ǵ�0������
                end else if (LOCAL_NODE_INFO[2]) begin
                    send_type_cnt <= 4'd2;  // ��һ���ǵ�2������
                end else if (LOCAL_NODE_INFO[3]) begin
                    send_type_cnt <= 4'd3;  // ��һ���ǵ�3������
                end else begin
                    send_type_cnt <= 4'd0;  // ֻ�е�1�����ݣ������ڵ�ǰ״̬
                end
            end
            
            4'd1: begin  // ��1�����ݣ�LOCAL_NODE_INFO[0]��Ӧ���������ͣ�
                if (LOCAL_NODE_INFO[0]) begin
                    // ���������ӵ�0�����ݵĴ����߼�
                    // ��ǰֻ��ռλ������������
                end
                
                // ״̬ת������ת����һ����Ч״̬
                if (LOCAL_NODE_INFO[1]) begin
                    send_type_cnt <= 4'd0;  // �ص�HDMI����
                end else if (LOCAL_NODE_INFO[2]) begin
                    send_type_cnt <= 4'd2;  // ��һ���ǵ�2������
                end else if (LOCAL_NODE_INFO[3]) begin
                    send_type_cnt <= 4'd3;  // ��һ���ǵ�3������
                end else begin
                    send_type_cnt <= 4'd1;  // ֻ�е�0�����ݣ������ڵ�ǰ״̬
                end
            end
            
            4'd2: begin  // ��2�����ݣ�LOCAL_NODE_INFO[2]��Ӧ���������ͣ�
                if (LOCAL_NODE_INFO[2]) begin
                    // ���������ӵ�2�����ݵĴ����߼�
                    // ��ǰֻ��ռλ������������
                end
                
                // ״̬ת������ת����һ����Ч״̬
                if (LOCAL_NODE_INFO[1]) begin
                    send_type_cnt <= 4'd0;  // �ص�HDMI����
                end else if (LOCAL_NODE_INFO[0]) begin
                    send_type_cnt <= 4'd1;  // �ص���0������
                end else if (LOCAL_NODE_INFO[3]) begin
                    send_type_cnt <= 4'd3;  // ��һ���ǵ�3������
                end else begin
                    send_type_cnt <= 4'd2;  // ֻ�е�2�����ݣ������ڵ�ǰ״̬
                end
            end
            
            4'd3: begin  // ��3�����ݣ�LOCAL_NODE_INFO[3]��Ӧ���������ͣ�
                if (LOCAL_NODE_INFO[3]) begin
                    // ���������ӵ�3�����ݵĴ����߼�
                    // ��ǰֻ��ռλ������������
                end
                
                // ״̬ת������ת����һ����Ч״̬
                if (LOCAL_NODE_INFO[1]) begin
                    send_type_cnt <= 4'd0;  // �ص�HDMI����
                end else if (LOCAL_NODE_INFO[0]) begin
                    send_type_cnt <= 4'd1;  // �ص���0������
                end else if (LOCAL_NODE_INFO[2]) begin
                    send_type_cnt <= 4'd2;  // �ص���2������
                end else begin
                    send_type_cnt <= 4'd3;  // ֻ�е�3�����ݣ������ڵ�ǰ״̬
                end
            end
            
            default: begin
                send_type_cnt <= 4'd0;
            end
        endcase
        
        // ��LOCAL_NODE_INFOȫΪ0ʱ�����ͽڵ���Ϣ
        if (LOCAL_NODE_INFO[3:0] == 4'h0) begin
            axis_to_sfp_tvalid <= 1'b1;
            axis_to_sfp_tdata <= {56'h0, LOCAL_NODE_INFO[7:4], 4'h0};
            axis_to_sfp_tlast <= 1'b1;
            axis_to_sfp_tkeep <= 8'h01;
        end
    end
end
// �����źţ�����ʱ������8ʱ����
assign hdmi_clear_all = hdmi_boundary_detected && (hdmi_boundary_delay_cnt == 4'd7);

//==========================================================================
// SFP�����߼���156.25MHzʱ����
//==========================================================================
always @(posedge tx_clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        // ��λʱ������������ź�
        hdmi_axi_rx_valid <= 1'b0;
        hdmi_axi_rx_data <= 64'h0;
        // hdmi_axi_rx_keep <= 8'h0;
    end else begin
        if (sfp_to_axis_tvalid) begin
            // ������������ֶΣ��������ͷַ�����
            case (sfp_to_axis_tdata[3:0])  // �����������(��4λ)
                DATA_TYPE_HDMI: begin  // HDMI��������
                    hdmi_axi_rx_valid <= 1'b1;
                    hdmi_axi_rx_data <= sfp_to_axis_tdata;
                    // hdmi_axi_rx_keep <= sfp_to_axis_tkeep;
                end
                DATA_TYPE_ETH: begin   // ��̫����������(Ԥ��)
                    // ��ʱ��������̫������
                    hdmi_axi_rx_valid <= 1'b0;
                end
                default: begin         // δ֪��������
                    hdmi_axi_rx_valid <= 1'b0;
                end
            endcase
        end else begin
            // û����Ч����ʱ���������
            hdmi_axi_rx_valid <= 1'b0;
        end
    end
end

// ��̫��������
xxv_ethernet_0 u_xxv_ethernet_0 (
  //��ڲ������
  .gt_rxp_in_0              (rxp_in),    // input wire gt_rxp_in_0
  .gt_rxn_in_0              (rxn_in),    // input wire gt_rxn_in_0
  .gt_txp_out_0             (txp_out),   // output wire gt_txp_out_0
  .gt_txn_out_0             (txn_out),   // output wire gt_txn_out_0

  .tx_clk_out_0             (tx_clk_out),   // output wire tx_clk_out_0
  .tx_reset_0               (gt0_rst),    // input wire tx_reset_0
  .rx_clk_out_0             ( ),    // output wire rx_clk_out_0
  .rx_reset_0               (gt0_rst),    // input wire rx_reset_0
  .rx_core_clk_0            (tx_clk_out),   // input wire rx_core_clk_0
  
  .txoutclksel_in_0         (3'b101),   // input wire [2 : 0] txoutclksel_in_0
  .rxoutclksel_in_0         (3'b101),   // input wire [2 : 0] rxoutclksel_in_0
  
  .gtwiz_reset_tx_datapath_0(1'b0), // input wire gtwiz_reset_tx_datapath_0
  .gtwiz_reset_rx_datapath_0(gtwiz_reset_rx_datapath),  // input wire gtwiz_reset_rx_datapath_0
  
  .rxrecclkout_0            ( ),    // output wire rxrecclkout_0
  
  .sys_reset                (~rst_n),   // input wire sys_reset
  .user_tx_reset_0          (tx_reset), // output wire user_tx_reset_0
 
  .dclk                     (sys_clk),     // input wire dclk
  .gt_loopback_in_0         (3'b000),   // input wire [2 : 0] gt_loopback_in_0
  .qpllreset_in_0           (1'b0),      // input wire qpllreset_in_0

  //��ڲο�ʱ��
  .gt_refclk_p              (q0_ck1_p_in),  // input wire gt_refclk_p
  .gt_refclk_n              (q0_ck1_n_in),  // input wire gt_refclk_n
  
  .gt_refclk_out            (gt_refclk_out),    // output wire gt_refclk_out
  .gtpowergood_out_0        (gtpowergood_out_0),    // output wire gtpowergood_out_0
  .user_rx_reset_0          (rx_reset),  // output wire user_rx_reset_0
  
  //AXI4 Stream ����ӿ��ź�
  .tx_axis_tready_0        (axis_to_sfp_tready),
  .tx_axis_tvalid_0        (axis_to_sfp_tvalid),
  .tx_axis_tdata_0         (axis_to_sfp_tdata),
  .tx_axis_tlast_0         (axis_to_sfp_tlast), 
  .tx_axis_tkeep_0         (axis_to_sfp_tkeep),
  .tx_axis_tuser_0         (1'b0),
  .tx_preamblein_0         (56'b0),            // input wire [55 : 0] tx_preamblein_0
  
  //RX �����ź�
  .ctl_rx_enable_0                  (1'b1), // input wire ctl_rx_enable_0
  .ctl_rx_check_preamble_0          (1'b0), // input wire ctl_rx_check_preamble_0
  .ctl_rx_check_sfd_0               (1'b0), // input wire ctl_rx_check_sfd_0
  .ctl_rx_force_resync_0            (1'b0), // input wire ctl_rx_force_resync_0
  .ctl_rx_delete_fcs_0              (1'b0), // input wire ctl_rx_delete_fcs_0
  .ctl_rx_ignore_fcs_0              (1'b1), // input wire ctl_rx_ignore_fcs_0
  .ctl_rx_max_packet_len_0          (15'h7FFF), // input wire [14 : 0] ctl_rx_max_packet_len_0
  .ctl_rx_min_packet_len_0          (15'd8 ),  // input wire [7 : 0] ctl_rx_min_packet_len_0
  .ctl_rx_process_lfi_0             (1'b0), // input wire ctl_rx_process_lfi_0
  .ctl_rx_test_pattern_0            (1'b0), // input wire ctl_rx_test_pattern_0
  .ctl_rx_data_pattern_select_0     (1'b0), // input wire ctl_rx_data_pattern_select_0
  .ctl_rx_test_pattern_enable_0     (1'b0), // input wire ctl_rx_test_pattern_enable_0
  .ctl_rx_custom_preamble_enable_0  (1'b0), // input wire ctl_rx_custom_preamble_enable_0
  
  //RX ����״̬�ź�
  .stat_rx_framing_err_0            ( ),    // output wire stat_rx_framing_err_0
  .stat_rx_framing_err_valid_0      ( ),    // output wire stat_rx_framing_err_valid_0
  .stat_rx_local_fault_0            ( ),    // output wire stat_rx_local_fault_0
  .stat_rx_block_lock_0             (),    // output wire stat_rx_block_lock_0
  .stat_rx_valid_ctrl_code_0        ( ),    // output wire stat_rx_valid_ctrl_code_0
  .stat_rx_status_0                 (stat_rx_status),   // output wire stat_rx_status_0
  .stat_rx_remote_fault_0           ( ),    // output wire stat_rx_remote_fault_0
  .stat_rx_bad_fcs_0                ( ),    // output wire [1 : 0] stat_rx_bad_fcs_0
  .stat_rx_stomped_fcs_0            ( ),    // output wire [1 : 0] stat_rx_stomped_fcs_0
  .stat_rx_truncated_0              ( ),    // output wire stat_rx_truncated_0
  .stat_rx_internal_local_fault_0   ( ),    // output wire stat_rx_internal_local_fault_0
  .stat_rx_received_local_fault_0   ( ),    // output wire stat_rx_received_local_fault_0
  .stat_rx_hi_ber_0                 ( ),    // output wire stat_rx_hi_ber_0
  .stat_rx_got_signal_os_0          ( ),    // output wire stat_rx_got_signal_os_0
  .stat_rx_test_pattern_mismatch_0  ( ),    // output wire stat_rx_test_pattern_mismatch_0
  .stat_rx_total_bytes_0            ( ),    // output wire [3 : 0] stat_rx_total_bytes_0
  .stat_rx_total_packets_0          ( ),    // output wire [1 : 0] stat_rx_total_packets_0
  .stat_rx_total_good_bytes_0       ( ),    // output wire [13 : 0] stat_rx_total_good_bytes_0
  .stat_rx_total_good_packets_0     ( ),    // output wire stat_rx_total_good_packets_0
  .stat_rx_packet_bad_fcs_0         ( ),    // output wire stat_rx_packet_bad_fcs_0
  .stat_rx_packet_64_bytes_0        ( ),    // output wire stat_rx_packet_64_bytes_0
  .stat_rx_packet_65_127_bytes_0    ( ),    // output wire stat_rx_packet_65_127_bytes_0
  .stat_rx_packet_128_255_bytes_0   ( ),    // output wire stat_rx_packet_128_255_bytes_0
  .stat_rx_packet_256_511_bytes_0   ( ),    // output wire stat_rx_packet_256_511_bytes_0
  .stat_rx_packet_512_1023_bytes_0  ( ),    // output wire stat_rx_packet_512_1023_bytes_0
  .stat_rx_packet_1024_1518_bytes_0 ( ),    // output wire stat_rx_packet_1024_1518_bytes_0
  .stat_rx_packet_1519_1522_bytes_0 ( ),    // output wire stat_rx_packet_1519_1522_bytes_0
  .stat_rx_packet_1523_1548_bytes_0 ( ),    // output wire stat_rx_packet_1523_1548_bytes_0
  .stat_rx_packet_1549_2047_bytes_0 ( ),    // output wire stat_rx_packet_1549_2047_bytes_0
  .stat_rx_packet_2048_4095_bytes_0 ( ),    // output wire stat_rx_packet_2048_4095_bytes_0
  .stat_rx_packet_4096_8191_bytes_0 ( ),    // output wire stat_rx_packet_4096_8191_bytes_0
  .stat_rx_packet_8192_9215_bytes_0 ( ),    // output wire stat_rx_packet_8192_9215_bytes_0
  .stat_rx_packet_small_0           ( ),    // output wire stat_rx_packet_small_0
  .stat_rx_packet_large_0           ( ),    // output wire stat_rx_packet_large_0
  .stat_rx_unicast_0                ( ),    // output wire stat_rx_unicast_0
  .stat_rx_multicast_0              ( ),    // output wire stat_rx_multicast_0
  .stat_rx_broadcast_0              ( ),    // output wire stat_rx_broadcast_0
  .stat_rx_oversize_0               ( ),    // output wire stat_rx_oversize_0
  .stat_rx_toolong_0                ( ),    // output wire stat_rx_toolong_0
  .stat_rx_undersize_0              ( ),    // output wire stat_rx_undersize_0
  .stat_rx_fragment_0               ( ),    // output wire stat_rx_fragment_0
  .stat_rx_vlan_0                   (  ),   // output wire stat_rx_vlan_0
  .stat_rx_inrangeerr_0             ( ),    // output wire stat_rx_inrangeerr_0
  .stat_rx_jabber_0                 ( ),    // output wire stat_rx_jabber_0
  .stat_rx_bad_code_0               ( ),    // output wire stat_rx_bad_code_0
  .stat_rx_bad_sfd_0                ( ),    // output wire stat_rx_bad_sfd_0
  .stat_rx_bad_preamble_0           ( ),    // output wire stat_rx_bad_preamble_0
  
  //AXI4 Stream ���սӿ��ź�
  .rx_axis_tvalid_0        (sfp_to_axis_tvalid),
  .rx_axis_tdata_0         (sfp_to_axis_tdata), 
  .rx_axis_tlast_0         (sfp_to_axis_tlast),
  .rx_axis_tkeep_0         (sfp_to_axis_tkeep),
  .rx_axis_tuser_0         (1'b0),
  
  .tx_unfout_0                    ( ),                // output wire tx_unfout_0

  //TX ״̬�ź�
  .stat_tx_local_fault_0            ( ),        // output wire stat_tx_local_fault_0
  .stat_tx_total_bytes_0            ( ),        // output wire [3 : 0] stat_tx_total_bytes_0
  .stat_tx_total_packets_0          ( ),        // output wire stat_tx_total_packets_0
  .stat_tx_total_good_bytes_0       ( ),        // output wire [13 : 0] stat_tx_total_good_bytes_0
  .stat_tx_total_good_packets_0     ( ),        // output wire stat_tx_total_good_packets_0
  .stat_tx_bad_fcs_0                ( ),        // output wire stat_tx_bad_fcs_0
  .stat_tx_packet_64_bytes_0        ( ),        // output wire stat_tx_packet_64_bytes_0
  .stat_tx_packet_65_127_bytes_0    ( ),        // output wire stat_tx_packet_65_127_bytes_0
  .stat_tx_packet_128_255_bytes_0   ( ),        // output wire stat_tx_packet_128_255_bytes_0
  .stat_tx_packet_256_511_bytes_0   ( ),        // output wire stat_tx_packet_256_511_bytes_0
  .stat_tx_packet_512_1023_bytes_0  ( ),        // output wire stat_tx_packet_512_1023_bytes_0
  .stat_tx_packet_1024_1518_bytes_0 ( ),        // output wire stat_tx_packet_1024_1518_bytes_0
  .stat_tx_packet_1519_1522_bytes_0 ( ),        // output wire stat_tx_packet_1519_1522_bytes_0
  .stat_tx_packet_1523_1548_bytes_0 ( ),        // output wire stat_tx_packet_1523_1548_bytes_0
  .stat_tx_packet_1549_2047_bytes_0 ( ),        // output wire stat_tx_packet_1549_2047_bytes_0
  .stat_tx_packet_2048_4095_bytes_0 ( ),        // output wire stat_tx_packet_2048_4095_bytes_0
  .stat_tx_packet_4096_8191_bytes_0 ( ),        // output wire stat_tx_packet_4096_8191_bytes_0
  .stat_tx_packet_8192_9215_bytes_0 ( ),        // output wire stat_tx_packet_8192_9215_bytes_0
  .stat_tx_packet_small_0           ( ),        // output wire stat_tx_packet_small_0
  .stat_tx_packet_large_0           ( ),        // output wire stat_tx_packet_large_0
  .stat_tx_unicast_0                ( ),        // output wire stat_tx_unicast_0
  .stat_tx_multicast_0              ( ),        // output wire stat_tx_multicast_0
  .stat_tx_broadcast_0              ( ),        // output wire stat_tx_broadcast_0
  .stat_tx_vlan_0                   ( ),        // output wire stat_tx_vlan_0
  .stat_tx_frame_error_0            ( ),        // output wire stat_tx_frame_error_0
  
  //AXI4?Stream �ӿ� - TX ·�������źź�״̬�ź�
  .ctl_tx_enable_0                  (1'b1),     // input wire ctl_tx_enable_0
  .ctl_tx_send_rfi_0                (1'b0),     // input wire ctl_tx_send_rfi_0
  .ctl_tx_send_lfi_0                (1'b0),     // input wire ctl_tx_send_lfi_0
  .ctl_tx_send_idle_0               (1'b0),     // input wire ctl_tx_send_idle_0
  .ctl_tx_fcs_ins_enable_0          (1'b0),     // input wire ctl_tx_fcs_ins_enable_0
  .ctl_tx_ignore_fcs_0              (1'b1),     // input wire ctl_tx_ignore_fcs_0
  
  .ctl_tx_test_pattern_0            (1'b0),     // input wire ctl_tx_test_pattern_0
  .ctl_tx_test_pattern_enable_0     (1'b0),     // input wire ctl_tx_test_pattern_enable_0
  .ctl_tx_test_pattern_select_0     (1'b0),     // input wire ctl_tx_test_pattern_select_0
  .ctl_tx_data_pattern_select_0     (1'b0),     // input wire ctl_tx_data_pattern_select_0
  .ctl_tx_test_pattern_seed_a_0     (1'b0),     // input wire [57 : 0] ctl_tx_test_pattern_seed_a_0
  .ctl_tx_test_pattern_seed_b_0     (1'b0),     // input wire [57 : 0] ctl_tx_test_pattern_seed_b_0
  .ctl_tx_ipg_value_0               (4'd0),    // input wire [3 : 0] ctl_tx_ipg_value_0
  
  .ctl_tx_custom_preamble_enable_0  (1'b0)     // input wire ctl_tx_custom_preamble_enable_0
);
assign  tx_disable = 2'b00;                     //�򿪹��
endmodule 