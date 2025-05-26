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

// FIFO����
parameter  FIFO_WIDTH = 24;                         // FIFO���Ϊ24λ
parameter  FIFO_DEPTH = 1024;                       // FIFO���Ϊ1024

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
// �ڲ��źŶ���
//==========================================================================
// FIFO����ź�
reg              fifo_rst;                          // FIFO��λ�ź�
reg [3:0]        fifo_rst_cnt;                      // FIFO��λ������
reg              fifo_rd_en;                        // FIFO��ʹ��
wire [FIFO_WIDTH-1:0] fifo_dout;                    // FIFO������ݣ�24λ
wire             fifo_empty;                        // FIFO�ձ�־
reg [FIFO_WIDTH-1:0] buffer;                        // д�����ݻ��壬24λ
reg              buf_valid;                         // ����������Ч
reg              fifo_write_enable;                 // FIFOд��ʹ���ź�

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
    .probe8(fifo_dout),   // input wire [7:0]  probe6 
    .probe9(fifo_rd_en),   // input wire [7:0]  probe6 
    .probe10(video_de_posedge),   // input wire [7:0]  probe6 
    .probe11(video_de_negedge)   // input wire [7:0]  probe6 
    // .probe12(video_de_posedge),   // video_de�����ؼ��
    // .probe13(video_de_negedge)    // video_de�½��ؼ��
    // .probe12(test_valid),   // input wire [7:0]  probe6 
    // .probe13(test_data)    // input wire [0:0]  probe5 
);

//==========================================================================
// AXI���ݽ��պ�д��FIFO (156.25MHzʱ����)
//==========================================================================
// FIFOʵ������24λ��1024���
fifo_generator_1 u_fifo_generator_1 (
    .rst           (fifo_rst),
    .wr_clk        (axi_clk),              // дʱ��ΪAXIʱ�� 156.25MHz
    .rd_clk        (pixel_clk),            // ��ʱ��Ϊ����ʱ�� 148.5MHz
    .din           (buffer),
    .wr_en         (buf_valid),
    .rd_en         (fifo_rd_en),
    .dout          (fifo_dout),
    .full          (),
    .empty         (fifo_empty),
    .wr_rst_busy   (),
    .rd_rst_busy   ()
);

integer i;

// ���ݽ��պ�д��
always @(posedge axi_clk or negedge rst_n) begin
    if(!rst_n) begin
        fifo_rst <= 1'b1;                 // ��λFIFO
        fifo_rst_cnt <= 4'd8;             // ���ø�λ������Ϊ8������
        buffer <= {FIFO_WIDTH{1'b0}};     // �������ݻ���
        buf_valid <= 1'b0;
        pixel1_data <= 24'd0;
        pixel2_data <= 24'd0;
        pixel1_valid <= 1'b0;
        pixel2_valid <= 1'b0;
        boundary_detected <= 1'b0;
        boundary_ack_sync_1 <= 1'b0;
        boundary_ack_sync_2 <= 1'b0;
        vs_negedge_sync_1 <= 1'b0;        // ��ʼ��VS�½���ͬ���Ĵ���
        vs_negedge_sync_2 <= 1'b0;        // ��ʼ��VS�½���ͬ���Ĵ���
        vs_negedge_axi <= 1'b0;           // ��ʼ��AXI��VS�½����ź�
        prev_video_de <= 1'b0;  // ��ʼ��ǰһ��������Ƶʹ���ź�
    end else begin
        // ��λFIFO�߼�
        if(fifo_rst) begin
            if(fifo_rst_cnt > 4'd0)
                fifo_rst_cnt <= fifo_rst_cnt - 4'd1;  // �������ݼ�
            else begin
                fifo_rst <= 1'b0;            // �����������ͷŸ�λ�ź�
                fifo_rst_cnt <= 4'd8;        // ���ü�������Ϊ�´θ�λ׼��
            end
        end
            
        // ͬ��VS�½��ص�AXIʱ����
        vs_negedge_sync_1 <= video_vs_negedge;
        vs_negedge_sync_2 <= vs_negedge_sync_1;
        vs_negedge_axi <= vs_negedge_sync_2;  // AXI���VS�½����ź�
            
        // ����⵽VS�½��ػ�video_de�½���ʱ����λFIFO
        if((vs_negedge_axi || video_de_negedge) && !fifo_rst)
            fifo_rst <= 1'b1;  // ��ʼ��λ����
        
        if (vs_negedge_axi) begin
            fifo_write_enable <= 1'b0;
        end
        
        // Ĭ������½���дʹ��
        buf_valid <= 1'b0;
            
        // �߽�ȷ���źŵĿ�ʱ����ͬ��(pixel_clk -> axi_clk)
        boundary_ack_sync_1 <= boundary_ack;
        boundary_ack_sync_2 <= boundary_ack_sync_1;
        
        // �յ�ȷ���źź�����߽����־
        if(boundary_ack_sync_2)
            boundary_detected <= 1'b0;
        
        // ������Ч����������
        if(hdmi_axi_rx_valid) begin
            // ����Ƿ�Ϊ�߽�����
            if(hdmi_axi_rx_data[63:16] == VS_ON_BOUNDARY_DATA && hdmi_axi_rx_data[15:14] == BOUNDARY_FLAG) begin
                // ���ֱ߽����ݣ����ñ߽����־ - ��д��FIFO
                boundary_detected <= 1'b1;
                fifo_write_enable <= 1'b1;
                // ���������Ч��־
                pixel1_valid <= 1'b0;
                pixel2_valid <= 1'b0;
            end else begin
                // �Ǳ߽����ݣ���ȡ���ز�������Ч��־д��FIFO
                // ����һ�������Ƿ���Ч��λ[15:14]�Ķ�Ӧλ��
                if(hdmi_axi_rx_data[15:14] == 2'b01 || hdmi_axi_rx_data[15:14] == 2'b10) begin
                    pixel1_data <= hdmi_axi_rx_data[39:16];  // �洢��һ������
                    pixel1_valid <= 1'b1;                    // ��ǵ�һ��������Ч
                    
                    // д���һ�����ص�FIFO
                    buffer <= hdmi_axi_rx_data[39:16];       // д���һ������
                    buf_valid <= 1'b1;                       // ʹ��FIFOд��
                end
                
                // ���ڶ��������Ƿ���Ч��˫����ģʽ��
                if(hdmi_axi_rx_data[15:14] == 2'b10) begin
                    pixel2_data <= hdmi_axi_rx_data[63:40];  // �洢�ڶ�������
                    pixel2_valid <= 1'b1;                    // ��ǵڶ���������Ч
                end else begin
                    pixel2_valid <= 1'b0;                    // �ڶ���������Ч
                end
            end
        end else if(pixel2_valid && fifo_write_enable) begin
            // �ڶ���������Ч������һ������д��FIFO
            buffer <= pixel2_data;        // д��ڶ�������
            buf_valid <= 1'b1;            // ʹ��FIFOд��
            pixel2_valid <= 1'b0;         // ����ڶ���������Ч��־
        end
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
always @(posedge pixel_clk or negedge rst_n) begin
    if(!rst_n) begin
        fifo_rd_en <= 1'b0;
        cnt_h <= 12'd0;
        cnt_v <= 12'd0;
        delay_cnt <= 8'd0;
        sync_enable <= 1'b0;
        video_hs <= 1'b0;
        video_vs <= 1'b0;
        video_de <= 1'b0;
        video_de_posedge <= 1'b0;
        video_de_negedge <= 1'b0;
        video_vs_prev <= 1'b0;        // ��ʼ��ǰһ�����ڵ�VS״̬
        video_vs_negedge <= 1'b0;     // ��ʼ��VS�½��ؼ��
        video_rgb <= 24'd0;
        send_state <= 2'd0;
    end else begin
        // Ĭ�Ͻ���FIFO��ʹ��
        fifo_rd_en <= 1'b0;
        
        // ����ǰһ���ڵ�video_vsֵ
        video_vs_prev <= video_vs;
        
        // ���video_vs���½���
        video_vs_negedge <= !video_vs && video_vs_prev;  // �½��أ���ǰΪ0��ǰһ����Ϊ1
        
        // ����ǰһ���ڵ�video_deֵ
        prev_video_de <= video_de;
        
        // ���video_de�������غ��½���
        video_de_posedge <= video_de && !prev_video_de;  // �����أ���ǰΪ1��ǰһ����Ϊ0
        video_de_negedge <= !video_de && prev_video_de;  // �½��أ���ǰΪ0��ǰһ����Ϊ1
        
        // ��video_de������ʱ��Ԥ����һ������
        if(video_de_posedge && !fifo_empty) begin
            fifo_rd_en <= 1'b1;  // ʹ��FIFO��ȡ
        end
        
        // �򻯵����ݶ�ȡ�߼�
        if(video_de) begin
            video_rgb <= fifo_dout;  // ֱ�ӽ�FIFO���������RGB
            fifo_rd_en <= 1'b1;      // ʹ��FIFO��ȡ��һ������
        end else begin
            // video_de��Чʱ�����ɫ
            video_rgb <= 24'd0;
            fifo_rd_en <= 1'b0;
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
        
       // ����ͬ���źź�����ʹ�� - ֻ����sync_enable��Чʱ������
        if(sync_enable) begin
            // ��ͬ���ź�����
            video_hs <= (cnt_h < H_SYNC) ? 1'b0 : 1'b1;  //��ͬ���źŸ�ֵ
            
            // ��ͬ���ź�����
            video_vs <= (cnt_v < V_SYNC) ? 1'b0 : 1'b1;  //��ͬ���źŸ�ֵ
            
            // ����ʹ���ź�����
            video_de <= ((cnt_h >= H_SYNC + H_BACK) && (cnt_h < H_SYNC + H_BACK + H_DISP)) && 
                         ((cnt_v >= V_SYNC + V_BACK) && (cnt_v < V_SYNC + V_BACK + V_DISP));
        end else begin
            // δʹ��ʱ�ر������ź�
            video_hs <= 1'b0;
            video_vs <= 1'b0;
            video_de <= 1'b0;
        end
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
endmodule