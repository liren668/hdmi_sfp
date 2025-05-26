//----------------------------------------------------------------------------------------
// File name:           axi_to_hdmi.v
// Created by:          ɺ����˹��
// Created date:        2025/05/13 16:00:00
// Version:             V0.1
// Descriptions:        AXI��HDMI���ݴ���ģ��
//                      ����hdmi_to_axi��AXI���ݣ��������ݷ��͸�HDMI��ʾ�ӿ�
//----------------------------------------------------------------------------------------

module axi_to_hdmi(
    // ϵͳ�ź�
    input                   rst_n,                  // ��λ�źţ��͵�ƽ��Ч
    input                   axi_clk,                // AXIʱ�� 156.25MHz
    input                   pixel_clk,              // ����ʱ�� 148.5MHz
    input                   clk_ila,                // ila����ʱ�� 300MHz
    
    // AXI�ӿڣ���hdmi_to_axi��������
    input                   hdmi_axi_rx_valid,      // ������Ч�ź�
    input      [63:0]       hdmi_axi_rx_data,       // ��������
    
    // HDMI����ӿ�
    output reg          video_hs,               // ��ͬ���ź�
    output reg          video_vs,               // ��ͬ���ź�
    output reg          video_de,               // ����ʹ��
    output reg [23:0]   video_rgb               // RGB888��ɫ����
);

//==========================================================================
// ��������
//==========================================================================
//������ʱ
parameter  BOUNDARY_DELAY = 16;                      // �߽�������ʱ

// �߽����ݶ���
parameter  VS_ON_BOUNDARY_DATA = 48'hFEFEFEFEFEFE;  // VS��1����߽�����,vs_in�źš�
parameter  BOUNDARY_FLAG = 2'b11;                   // �߽����ݱ�־

// ʱ�����
parameter  H_SYNC   =  12'd44;    // ��ͬ��
parameter  H_BACK   =  12'd148;   // ����ʾ����
parameter  H_DISP   =  12'd1920;  // ����Ч����
parameter  H_FRONT  =  12'd88;    // ����ʾǰ��
parameter  H_TOTAL  =  12'd2200;  // ��ɨ������

parameter  V_SYNC   =  12'd5;     // ��ͬ��
parameter  V_BACK   =  12'd36;    // ����ʾ����
parameter  V_DISP   =  12'd1080;  // ����Ч����
parameter  V_FRONT  =  12'd4;     // ����ʾǰ��
parameter  V_TOTAL  =  12'd1125;  // ��ɨ������

//==========================================================================
// ���ζ��в������źŶ���
//==========================================================================
localparam RING_DEPTH = 128;              // �������128
localparam RING_ADDR_WIDTH = 7;           // ��ַ���7λ
reg [23:0] ring_data[0:RING_DEPTH-1];     // 24λ��128��
reg [RING_ADDR_WIDTH-1:0] wr_ptr;         // дָ�루axi_clk��
reg [RING_ADDR_WIDTH-1:0] rd_ptr;         // ��ָ�루pixel_clk��
wire ring_full;
wire ring_empty;
reg [RING_ADDR_WIDTH-1:0] next_wr_ptr;
reg [RING_ADDR_WIDTH-1:0] wr_ptr_sync1, wr_ptr_sync2; // дָ�����ͬ����pixel_clk
reg [RING_ADDR_WIDTH-1:0] rd_ptr_sync1, rd_ptr_sync2; // ��ָ�����ͬ����axi_clk

// ���ݽ��ռĴ���
reg [23:0]       pixel1_data;                       // ��һ����������
reg [23:0]       pixel2_data;                       // �ڶ�����������
reg              pixel1_valid;                      // ��һ��������Ч
reg              pixel2_valid;                      // �ڶ���������Ч

// �߽����ݼ���ͬ��
reg             boundary_detected;                 // AXI��߽����־
(* ASYNC_REG = "TRUE" *) reg  boundary_sync_1;     // ��ʱ����ͬ���Ĵ���1
(* ASYNC_REG = "TRUE" *) reg  boundary_sync_2;     // ��ʱ����ͬ���Ĵ���2
reg             boundary_pulse;                    // ͬ������
reg             boundary_ack;                      // �߽�����ȷ���ź�
(* ASYNC_REG = "TRUE" *) reg  boundary_ack_sync_1; // ȷ���ź�ͬ���Ĵ���1
(* ASYNC_REG = "TRUE" *) reg  boundary_ack_sync_2; // ȷ���ź�ͬ���Ĵ���2

// �������Ϳ����ź�
reg [11:0]      cnt_h;          // �м�����
reg [11:0]      cnt_v;          // ��������
reg [7:0]       delay_cnt;      // ��ʱ������
reg             sync_enable;    // ͬ��ʹ���ź�
integer         i;              // ѭ�������������ڳ�ʼ�����ζ���

// �����ݶ�ȡ�ʹ����֣����Ԥ������
reg             prev_video_de;         // ǰһ�����ڵ�video_de״̬

// ���ݶ�ȡ������߼�
reg [1:0] send_state; // 0:��ʼ״̬ 1:���͵�һ������ 2:���͵ڶ�������

// ���ؼ���ź�
reg video_de_posedge;  // video_de�����ؼ��
reg video_de_negedge;  // video_de�½��ؼ��
reg video_vs_prev;     // ǰһ�����ڵ�video_vs״̬
reg video_vs_negedge;  // video_vs�½��ؼ��
reg vs_negedge_sync_1; // VS�½���ͬ���Ĵ���1
reg vs_negedge_sync_2; // VS�½���ͬ���Ĵ���2
reg vs_negedge_axi;    // AXIʱ�����VS�½����ź�
(* ASYNC_REG = "TRUE" *) reg de_negedge_sync_1; // DE�½���ͬ���Ĵ���1
(* ASYNC_REG = "TRUE" *) reg de_negedge_sync_2; // DE�½���ͬ���Ĵ���2
reg de_negedge_axi;    // AXIʱ�����DE�½����ź�

// ���״̬������
localparam IDLE = 2'b00;      // ����״̬
localparam WRITE_PIXEL1 = 2'b01;  // д���һ������
localparam WRITE_PIXEL2 = 2'b10;  // д��ڶ�������

reg [1:0] write_state;  // д��״̬��
reg [63:0] pixel_data_reg;  // ��ʱ�洢��������

//ILAʵ����
ila_2 u_ila_2 (
    .clk(clk_ila),              // input wire clk300Mhz
    .probe0(pixel_clk),   // input wire [63:0]  probe0  
    .probe1(video_vs),  // input wire [7:0]  probe1 
    .probe2(video_hs),   // input wire [0:0]  probe2 
    .probe3(video_de),     // input wire [63:0]  probe3 
    .probe4(video_rgb),    // input wire [0:0]  probe4 

    .probe5(hdmi_axi_rx_valid),  // input wire [7:0]  probe6 
    .probe6(hdmi_axi_rx_data),  // input wire [7:0]  probe6 

    .probe7(boundary_detected),   // input wire [7:0]  probe6 
    .probe8(wr_ptr),   // input wire [7:0]  probe6 
    .probe9(rd_ptr),   // input wire [7:0]  probe6 
    .probe10(video_de_posedge),   // input wire [7:0]  probe6 
    .probe11(video_de_negedge)   // input wire [7:0]  probe6 
    // .probe12(video_de_posedge),   // video_de�����ؼ��
    // .probe13(video_de_negedge)    // video_de�½��ؼ��
    // .probe12(test_valid),   // input wire [7:0]  probe6 
    // .probe13(test_data)    // input wire [0:0]  probe5 
);

//==========================================================================
// ����״̬�ж�
//==========================================================================
always @(*) begin
    next_wr_ptr = (wr_ptr == RING_DEPTH-1) ? 0 : wr_ptr + 1'b1;
end
assign ring_full = (next_wr_ptr == rd_ptr_sync2);
assign ring_empty = (rd_ptr == wr_ptr_sync2);

//==========================================================================
// дָ���ʱ����ͬ����pixel_clk
//==========================================================================
always @(posedge pixel_clk or negedge rst_n) begin
    if (!rst_n) begin wr_ptr_sync1 <= 0; wr_ptr_sync2 <= 0; end
    else begin wr_ptr_sync1 <= wr_ptr; wr_ptr_sync2 <= wr_ptr_sync1; end
end
// ��ָ���ʱ����ͬ����axi_clk
always @(posedge axi_clk or negedge rst_n) begin
    if (!rst_n) begin rd_ptr_sync1 <= 0; rd_ptr_sync2 <= 0; end
    else begin rd_ptr_sync1 <= rd_ptr; rd_ptr_sync2 <= rd_ptr_sync1; end
end

//==========================================================================
// ���ζ���д���߼���AXIʱ����156.25MHz
//==========================================================================
reg write_enable;
always @(posedge axi_clk or negedge rst_n) begin
    if(!rst_n) begin
        write_enable <= 1'b0;
        wr_ptr <= 0;
        boundary_detected <= 1'b0;  // ��λʱ����
        write_state <= IDLE;
        pixel_data_reg <= 64'd0;
        for (i = 0; i < RING_DEPTH; i = i + 1)
            ring_data[i] <= 24'd0;
    end else begin
        case(write_state)
            IDLE: begin
                // ����Ƿ�Ϊ�߽�����
                if(hdmi_axi_rx_valid) begin
                    if(hdmi_axi_rx_data[63:16] == VS_ON_BOUNDARY_DATA && hdmi_axi_rx_data[15:14] == BOUNDARY_FLAG) begin
                        // �߽����ݼ���߼�
                        boundary_detected <= 1'b1;
                        write_enable <= 1'b1;
                        pixel1_valid <= 1'b0;
                        pixel2_valid <= 1'b0;
                    end else if (write_enable) begin
                        // �Ǳ߽����ݣ�׼��д�����ص����ζ���
                        case(hdmi_axi_rx_data[15:14])
                            2'b01: begin
                                ring_data[wr_ptr] <= hdmi_axi_rx_data[39:16];
                                wr_ptr <= (wr_ptr == RING_DEPTH-1) ? 0 : wr_ptr + 1'b1;
                            end
                            2'b10: begin
                                ring_data[wr_ptr] <= hdmi_axi_rx_data[39:16]; // ��д��һ������
                                ring_data[(wr_ptr == RING_DEPTH-1) ? 0 : wr_ptr + 1'b1] <= hdmi_axi_rx_data[63:40]; // ��д�ڶ�������
                                wr_ptr <= (wr_ptr == RING_DEPTH-2) ? 0 : (wr_ptr == RING_DEPTH-1) ? 1 : wr_ptr + 2'b10;
                            end
                        endcase
                    end
                end
            end

            WRITE_PIXEL1: begin
                // д���һ������
                ring_data[wr_ptr] <= pixel_data_reg[39:16];
                wr_ptr <= (wr_ptr == RING_DEPTH-1) ? 0 : wr_ptr + 1'b1;
                write_state <= WRITE_PIXEL2;  // ����д��ڶ�������״̬
            end

            WRITE_PIXEL2: begin
                // д��ڶ�������
                ring_data[wr_ptr] <= pixel_data_reg[63:40];
                wr_ptr <= (wr_ptr == RING_DEPTH-1) ? 0 : wr_ptr + 1'b1;
                write_state <= IDLE;  // ���ؿ���״̬
            end
        endcase

        // ���߽�ȷ���ź�ͬ������ʱ������߽����־
        if(boundary_ack_sync_2) begin
            boundary_detected <= 1'b0;  // ����߽����־
        end

        // video_de�½��ػ�VS�½���ʱ����дָ��Ͷ���
        if((vs_negedge_axi || de_negedge_axi)) begin
            wr_ptr <= 0;
            boundary_detected <= 1'b0;  // ͬ���źŵ���ʱҲ����
            write_state <= IDLE;  // ����״̬��
            for (i = 0; i < RING_DEPTH; i = i + 1)
                ring_data[i] <= 24'd0;
        end
        if(vs_negedge_axi) begin
            write_enable <= 1'b0;
            boundary_detected <= 1'b0;  // VS�½���ʱҲ����
            write_state <= IDLE;  // ����״̬��
        end
    end
end

//==========================================================================
// ���ζ��ж����߼�������ʱ����148.5MHz
//==========================================================================
always @(posedge pixel_clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_ptr <= 0;
        video_rgb <= 24'd0;
    end else begin
        if (video_de0) begin
                video_de <= 1'b1;
                video_rgb <= ring_data[rd_ptr];
                rd_ptr <= (rd_ptr == RING_DEPTH-1) ? 0 : rd_ptr + 1'b1;
        end else begin
            video_rgb <= 24'd0;
            video_de <= 1'b0;
        end
        // video_de�½��ػ�VS�½���ʱ�����ָ��
        if (video_de_negedge || video_vs_negedge)
            rd_ptr <= 0;
    end
end

//==========================================================================
// ��ʱ����ͬ�� (156.25MHz �� 148.5MHz)
//==========================================================================
always @(posedge pixel_clk or negedge rst_n) begin
    if(!rst_n) begin
        boundary_sync_1 <= 1'b0;
        boundary_sync_2 <= 1'b0;
        boundary_pulse <= 1'b0;
        boundary_ack <= 1'b0;
    end else begin
        // �߽��źſ�ʱ����ͬ��
        boundary_sync_1 <= boundary_detected;
        boundary_sync_2 <= boundary_sync_1;
        
        // �߽�ͬ���������� - �����ؼ��
        boundary_pulse <= boundary_sync_1 && !boundary_sync_2;
        
        // ����ȷ���ź� - ����⵽�߽�����ʱ
        if(boundary_pulse)
            boundary_ack <= 1'b1;  // ����ȷ���ź�
        else if(!boundary_sync_1)
            boundary_ack <= 1'b0;  // ���߽��ź��Ѿ�����ʱ�����ȷ���ź�
    end
end

//==========================================================================
// ���ݶ�ȡ�ʹ��� (148.5MHzʱ����)
//==========================================================================
reg video_de0;
always @(posedge pixel_clk or negedge rst_n) begin
    if(!rst_n) begin
        cnt_h <= 12'd0;
        cnt_v <= 12'd0;
        delay_cnt <= 8'd0;
        sync_enable <= 1'b0;
        video_hs <= 1'b0;
        video_vs <= 1'b0;
        video_de0 <= 1'b0;
        video_de_posedge <= 1'b0;
        video_de_negedge <= 1'b0;
        video_vs_prev <= 1'b0;        // ��ʼ��ǰһ�����ڵ�VS״̬
        video_vs_negedge <= 1'b0;     // ��ʼ��VS�½��ؼ��
        video_rgb <= 24'd0;
        send_state <= 2'd0;
    end else begin
        
        // ����ǰһ���ڵ�video_vsֵ
        video_vs_prev <= video_vs;
        
        // ���video_vs���½���
        video_vs_negedge <= !video_vs && video_vs_prev;  // �½��أ���ǰΪ0��ǰһ����Ϊ1
        
        // ����ǰһ���ڵ�video_deֵ
        prev_video_de <= video_de0;
        
        // ���video_de�������غ��½���
        video_de_posedge <= video_de && !prev_video_de;  // �����أ���ǰΪ1��ǰһ����Ϊ0
        video_de_negedge <= !video_de && prev_video_de;  // �½��أ���ǰΪ0��ǰһ����Ϊ1
        
        // �߽������� - ��AXI��ͬ�������ı߽�����
        if(boundary_pulse) begin
            if(!sync_enable) begin   // ���ͬ��δʹ��
                delay_cnt <= BOUNDARY_DELAY;  // ������ʱ��Ĭ��Ϊ8
            end
        end
        // ��ʱ��������
        if(!sync_enable && delay_cnt > 0) begin
            delay_cnt <= delay_cnt - 1'b1;
            if(delay_cnt == 8'd1) begin
                sync_enable <= 1'b1;     // ��ʱ����������ͬ���ź�����
            end
        end
        // ���ؼ��������� - ��sync_enable����
        if(sync_enable) begin
            if(cnt_h < H_TOTAL - 1'b1)
                cnt_h <= cnt_h + 1'b1;
            else begin
                cnt_h <= 12'd0;     
                if(cnt_v < V_TOTAL - 1'b1)
                    cnt_v <= cnt_v + 1'b1;
                else
                    cnt_v <= 12'd0;   
            end
        end
       // ����ͬ���źź�����ʹ�� - ֻ����sync_enable��Чʱ������
        if(sync_enable) begin
            // ��ͬ���ź�����
            video_hs <= (cnt_h < H_SYNC) ? 1'b0 : 1'b1;  //��ͬ���źŸ�ֵ
            
            // ��ͬ���ź�����
            video_vs <= (cnt_v < V_SYNC) ? 1'b0 : 1'b1;  //��ͬ���źŸ�ֵ
            
            // ����ʹ���ź�����
            video_de0 <= ((cnt_h >= H_SYNC + H_BACK) && (cnt_h < H_SYNC + H_BACK + H_DISP)) && 
                         ((cnt_v >= V_SYNC + V_BACK) && (cnt_v < V_SYNC + V_BACK + V_DISP));
        end else begin
            // δʹ��ʱ�ر������ź�
            video_hs <= 1'b0;
            video_vs <= 1'b0;
            video_de0 <= 1'b0;
        end
    end
end

//==========================================================================
// ���Ӷ�֡��⣺3֡��boundary_pulse��sync_enable����
//==========================================================================
reg [1:0] no_boundary_vs_cnt; // 2λ����������¼������boundary_pulse��VS������

// ��video_vs������ʱ�ж��Ƿ���boundary_pulse
always @(posedge pixel_clk or negedge rst_n) begin
    if (!rst_n) begin
        no_boundary_vs_cnt <= 2'd0;
    end else begin
        // video_vs�����ؼ��
        if (video_vs && !video_vs_prev) begin
            if (!boundary_pulse) begin
                if (no_boundary_vs_cnt < 2'd3)
                    no_boundary_vs_cnt <= no_boundary_vs_cnt + 1'b1;
            end else begin
                no_boundary_vs_cnt <= 2'd0; // ��⵽boundary_pulse������
            end
        end
        // ֻҪ��⵽boundary_pulse����ʱ���������
        if (boundary_pulse)
            no_boundary_vs_cnt <= 2'd0;
    end
end

// ������3֡��boundary_pulseʱ��sync_enable�Զ�����
always @(posedge pixel_clk or negedge rst_n) begin
    if (!rst_n) begin
        sync_enable <= 1'b0;
    end else if (no_boundary_vs_cnt == 2'd3) begin
        sync_enable <= 1'b0;
    end
end

//==========================================================================
// ��ʱ����ͬ�� (148.5MHz �� 156.25MHz)
//==========================================================================
always @(posedge axi_clk or negedge rst_n) begin
    if(!rst_n) begin
        vs_negedge_sync_1 <= 1'b0;
        vs_negedge_sync_2 <= 1'b0;
        vs_negedge_axi <= 1'b0;
    end else begin
        // ͬ��VS�½��ص�AXIʱ����
        vs_negedge_sync_1 <= video_vs_negedge;
        vs_negedge_sync_2 <= vs_negedge_sync_1;
        vs_negedge_axi <= vs_negedge_sync_2;
    end
end

//==========================================================================
// DE�½��ؿ�ʱ����ͬ�� (148.5MHz �� 156.25MHz)
//==========================================================================
always @(posedge axi_clk or negedge rst_n) begin
    if(!rst_n) begin
        de_negedge_sync_1 <= 1'b0;
        de_negedge_sync_2 <= 1'b0;
        de_negedge_axi <= 1'b0;
    end else begin
        // ͬ��DE�½��ص�AXIʱ����
        de_negedge_sync_1 <= video_de_negedge;
        de_negedge_sync_2 <= de_negedge_sync_1;
        de_negedge_axi <= de_negedge_sync_2;
    end
end
endmodule