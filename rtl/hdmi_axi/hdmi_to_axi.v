//----------------------------------------------------------------------------------------
// File name:           hdmi_to_axi.v
// Created by:          ɺ����˹��
// Created date:        2025.5
// Version:             V0.1
// Descriptions:        SFP��HDMI���ݴ���ģ��
//----------------------------------------------------------------------------------------

module hdmi_to_axi(
    // ϵͳ�ź�
    input                    rst_n,               // ��λ�źţ��͵�ƽ��Ч
    // input                    clk_ila,             // ILAʱ�� 300MHz
    //HDMI�����ź�
    input               init_over,           // HDMIоƬ��ʼ������źţ�����Ч
	input               video_clk_in    ,   // ����ʱ��148.5MHz                        
    input            	video_vs_in     ,   // ���볡ͬ���ź�
    input            	video_de_in     ,   // ����ʹ��
    input     [23:0]    video_rgb_in    ,   // RGB888��ɫ����
    
    // �����HDMI��AXI�ӿ�
    output reg               hdmi_axi_tx_valid,   // ������Ч�ź�
    output reg  [63:0]       hdmi_axi_tx_data     // �������
);

//==========================================================================
// ��������
//==========================================================================
// �ڵ���Ϣ����
localparam NODE_INFO = 8'h12;            // �ڵ���Ϣ���̶�Ϊ0x12
localparam BOUNDARY_DATA = {48'hFEFEFEFEFEFE, 2'b11, 6'd0, NODE_INFO};

//==========================================================================
// �ڲ��źŶ���
//==========================================================================
// HDMI�����źżĴ��������ڴ�һ�ģ�
reg             video_vs_in_d0;          // ���볡ͬ���źżĴ���
reg             video_vs_in_d1;          // ���볡ͬ���źżĴ���
reg             video_de_in_d0;          // ����ʹ�ܼĴ���
reg [23:0]      video_rgb_in_d0;         // RGB888��ɫ���ݼĴ���
reg             prev_de;                 // ǰһ��DE�źţ����ڼ�����
reg             vs_pose;                 // ���ź�������
reg             vs_nege;                 // ���ź��½��أ��������ź��½��ؼ��

// ����ƴ�Ӻʹ���
reg             frame_process_en;        // ֡����ʹ���ź�
reg [63:0]      buffer;                  // 64λƴ�����ݼĴ���
reg [1:0]       wr_cnt;                  // д��������0-1�����2��RGB��

//ILAʵ����
// ila_0 u_ila_0 (
//     .clk(clk_ila),              // input wire clk300Mhz
//     .probe0(video_clk_in),   // input wire [63:0]  probe0  
//     .probe1(video_vs_in_d0),  // input wire [7:0]  probe1 
//     .probe2(video_de_in_d0),   // input wire [0:0]  probe2 
//     .probe3(video_rgb_in_d0),    // input wire [23:0]  probe4 
//     .probe4(vs_nege),  // input wire [7:0]  probe6 
//     .probe5(hdmi_axi_tx_valid),   // input wire [7:0]  probe6 
//     .probe6(hdmi_axi_tx_data)  // input wire [7:0]  probe6 
//     // .probe7(),  // input wire [7:0]  probe6 
//     // .probe8(),   // input wire [7:0]  probe6 
//     // .probe9(),   // input wire [7:0]  probe6 
//     // .probe10()   // input wire [7:0]  probe6 
//     // .probe12(video_de_posedge),   // video_de�����ؼ��
//     // .probe13(video_de_negedge)    // video_de�½��ؼ��
//     // .probe12(test_valid),   // input wire [7:0]  probe6 
//     // .probe13(test_data)    // input wire [0:0]  probe5 
// );

//==========================================================================
// HDMI�����źŴ�һ�ģ�148.5MHzʱ����
//==========================================================================
integer         i;                       // ����ѭ����ʼ������������
always @(posedge video_clk_in) begin
    if(!init_over) begin                                ////HDMIоƬ��ʼ������źţ�����Ч
        video_vs_in_d0 <= 1'b0;
        video_vs_in_d1 <= 1'b0;
        video_de_in_d0 <= 1'b0;
        video_rgb_in_d0 <= 24'd0;
        prev_de <= 1'b0;
    end else begin
        // �������źŴ�һ��
        video_vs_in_d0 <= video_vs_in;
        video_vs_in_d1 <= video_vs_in_d0;
        video_de_in_d0 <= video_de_in;
        video_rgb_in_d0 <= video_rgb_in;
        prev_de <= video_de_in_d0;  // ��¼ǰһ��DE�ź�
    end
end

//�������ź������غ��½���
always@(posedge video_clk_in or negedge rst_n)begin
	if(!rst_n) begin
		vs_pose <= 1'b0;
		vs_nege <= 1'b0;  // ��ʼ�����ź��½��ر�־
	end else if(video_vs_in_d0 && ~video_vs_in_d1)
		vs_pose <= 1'b1;  // ��⵽��ͬ���źŵ������أ�����vs_poseΪ1
	else if(~video_vs_in_d0 && video_vs_in_d1)
		vs_nege <= 1'b1;  // ��⵽��ͬ���źŵ��½��أ�����vs_negeΪ1
    else begin
		vs_pose <= 1'b0;  // ��������ر�־��ȷ��ֻ����һ��ʱ������
		vs_nege <= 1'b0;  // ����½��ر�־��ȷ��ֻ����һ��ʱ������
	end
end

//==========================================================================
// ����ƴ�Ӵ���������148.5MHzʱ����
//==========================================================================
reg             frame_process_en1;        // ֡����ʹ���ź�
always @(posedge video_clk_in or negedge rst_n) begin
    if(!rst_n) begin  // ϵͳ��λʱ������buffer
        wr_cnt <= 2'd0;
        buffer <= 64'd0;
        hdmi_axi_tx_valid <= 1'b0;
        hdmi_axi_tx_data <= 64'd0;
        frame_process_en <= 1'b0;
        frame_process_en1 <= 1'b0;
    end else begin
        // Ĭ����������Ч��־
        hdmi_axi_tx_valid <= 1'b0;
        
        // ��ⳡ�ź������أ�����߽����ݣ�������ȼ���
        if(vs_nege) begin
            hdmi_axi_tx_valid <= 1'b1;
            hdmi_axi_tx_data <= BOUNDARY_DATA;
            frame_process_en <= 1'b0;
            frame_process_en1 <= 1'b1;
        end
        // ��ⳡͬ���ź������أ�����֡����
        if(frame_process_en1) begin
            frame_process_en <= 1'b1;  // ��⵽��ͬ���ź������أ�����֡����
            frame_process_en1 <= 1'b0;
        end
        // ����������Ƶ���ݣ�ƴ�ӽ׶Σ�
        else if(video_de_in_d0 && frame_process_en) begin
            // ������䣬ֱ�ӽ�RGB���ݷ��뻺����
            case(wr_cnt)
                2'd0: begin
                    buffer[39:16] <= video_rgb_in_d0; // ��һ��RGB���ݷ���[39:16]
                    wr_cnt <= wr_cnt + 1'b1;
                end
                2'd1: begin
                    buffer[63:40] <= video_rgb_in_d0; // �ڶ���RGB���ݷ���[63:40]
                    buffer[15:14] <= 2'd2;            // 2��ʾ�����ֽڶ���Ч
                    buffer[13:8] <= 6'd0;             // 6��0
                    buffer[7:0] <= NODE_INFO;         // �ڵ���Ϣ
                    wr_cnt <= 2'd0;
                    // ƴ����ɣ��������
                    hdmi_axi_tx_valid <= 1'b1;
                    hdmi_axi_tx_data <= {video_rgb_in_d0, buffer[39:16], 2'd2, 6'd0, NODE_INFO};
                end
            endcase
        end else if(!video_de_in_d0 && prev_de) begin
            // DE�½��ش���֡�������н�����
            if(wr_cnt == 2'd1) begin // ��δƴ����ɵ�����
                // ���ʣ�ಿ�ֲ����
                buffer[63:40] <= 24'd0;          // �ڶ���RGBλ����0
                buffer[15:14] <= 2'd1;            // 1��ʾֻ�е�һ��RGB������Ч
                wr_cnt <= 2'd0;
                // �������������
                hdmi_axi_tx_valid <= 1'b1;
                hdmi_axi_tx_data <= {24'd0, buffer[39:16], 2'd1, 6'd0, NODE_INFO};      // ������������ݣ�Ҳ������
            end else if(wr_cnt == 2'd0) begin
                // �н��������������־
                hdmi_axi_tx_valid <= 1'b1;
                hdmi_axi_tx_data <= {48'hf0f0f0f0f0f0, 2'd3, 6'd0, NODE_INFO};          // ���������־
            end 
        end
    end
end
endmodule 