//----------------------------------------------------------------------------------------
// File name:           hdmi_to_axi.v
// Created by:          珊瑚伊斯特
// Created date:        2025.5
// Version:             V0.1
// Descriptions:        SFP到HDMI数据处理模块
//----------------------------------------------------------------------------------------

module hdmi_to_axi(
    // 系统信号
    input                    rst_n,               // 复位信号，低电平有效
    // input                    clk_ila,             // ILA时钟 300MHz
    //HDMI输入信号
    input               init_over,           // HDMI芯片初始化完成信号，高有效
	input               video_clk_in    ,   // 输入时钟148.5MHz                        
    input            	video_vs_in     ,   // 输入场同步信号
    input            	video_de_in     ,   // 数据使能
    input     [23:0]    video_rgb_in    ,   // RGB888颜色数据
    
    // 输出到HDMI的AXI接口
    output reg               hdmi_axi_tx_valid,   // 数据有效信号
    output reg  [63:0]       hdmi_axi_tx_data     // 输出数据
);

//==========================================================================
// 参数定义
//==========================================================================
// 节点信息定义
localparam NODE_INFO = 8'h12;            // 节点信息，固定为0x12
localparam BOUNDARY_DATA = {48'hFEFEFEFEFEFE, 2'b11, 6'd0, NODE_INFO};

//==========================================================================
// 内部信号定义
//==========================================================================
// HDMI输入信号寄存器（用于打一拍）
reg             video_vs_in_d0;          // 输入场同步信号寄存器
reg             video_vs_in_d1;          // 输入场同步信号寄存器
reg             video_de_in_d0;          // 数据使能寄存器
reg [23:0]      video_rgb_in_d0;         // RGB888颜色数据寄存器
reg             prev_de;                 // 前一个DE信号，用于检测边沿
reg             vs_pose;                 // 场信号上升沿
reg             vs_nege;                 // 场信号下降沿，新增场信号下降沿检测

// 数据拼接和处理
reg             frame_process_en;        // 帧处理使能信号
reg [63:0]      buffer;                  // 64位拼接数据寄存器
reg [1:0]       wr_cnt;                  // 写计数器（0-1，最多2个RGB）

//ILA实例化
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
//     // .probe12(video_de_posedge),   // video_de上升沿检测
//     // .probe13(video_de_negedge)    // video_de下降沿检测
//     // .probe12(test_valid),   // input wire [7:0]  probe6 
//     // .probe13(test_data)    // input wire [0:0]  probe5 
// );

//==========================================================================
// HDMI输入信号打一拍（148.5MHz时钟域）
//==========================================================================
integer         i;                       // 用于循环初始化的索引变量
always @(posedge video_clk_in) begin
    if(!init_over) begin                                ////HDMI芯片初始化完成信号，高有效
        video_vs_in_d0 <= 1'b0;
        video_vs_in_d1 <= 1'b0;
        video_de_in_d0 <= 1'b0;
        video_rgb_in_d0 <= 24'd0;
        prev_de <= 1'b0;
    end else begin
        // 对输入信号打一拍
        video_vs_in_d0 <= video_vs_in;
        video_vs_in_d1 <= video_vs_in_d0;
        video_de_in_d0 <= video_de_in;
        video_rgb_in_d0 <= video_rgb_in;
        prev_de <= video_de_in_d0;  // 记录前一个DE信号
    end
end

//产生场信号上升沿和下降沿
always@(posedge video_clk_in or negedge rst_n)begin
	if(!rst_n) begin
		vs_pose <= 1'b0;
		vs_nege <= 1'b0;  // 初始化场信号下降沿标志
	end else if(video_vs_in_d0 && ~video_vs_in_d1)
		vs_pose <= 1'b1;  // 检测到场同步信号的上升沿，设置vs_pose为1
	else if(~video_vs_in_d0 && video_vs_in_d1)
		vs_nege <= 1'b1;  // 检测到场同步信号的下降沿，设置vs_nege为1
    else begin
		vs_pose <= 1'b0;  // 清除上升沿标志，确保只持续一个时钟周期
		vs_nege <= 1'b0;  // 清除下降沿标志，确保只持续一个时钟周期
	end
end

//==========================================================================
// 数据拼接处理和输出（148.5MHz时钟域）
//==========================================================================
reg             frame_process_en1;        // 帧处理使能信号
always @(posedge video_clk_in or negedge rst_n) begin
    if(!rst_n) begin  // 系统复位时都清零buffer
        wr_cnt <= 2'd0;
        buffer <= 64'd0;
        hdmi_axi_tx_valid <= 1'b0;
        hdmi_axi_tx_data <= 64'd0;
        frame_process_en <= 1'b0;
        frame_process_en1 <= 1'b0;
    end else begin
        // 默认清除输出有效标志
        hdmi_axi_tx_valid <= 1'b0;
        
        // 检测场信号上升沿，输出边界数据（最高优先级）
        if(vs_nege) begin
            hdmi_axi_tx_valid <= 1'b1;
            hdmi_axi_tx_data <= BOUNDARY_DATA;
            frame_process_en <= 1'b0;
            frame_process_en1 <= 1'b1;
        end
        // 检测场同步信号上升沿，启用帧处理
        if(frame_process_en1) begin
            frame_process_en <= 1'b1;  // 检测到场同步信号上升沿，启用帧处理
            frame_process_en1 <= 1'b0;
        end
        // 处理输入视频数据（拼接阶段）
        else if(video_de_in_d0 && frame_process_en) begin
            // 数据填充，直接将RGB数据放入缓冲区
            case(wr_cnt)
                2'd0: begin
                    buffer[39:16] <= video_rgb_in_d0; // 第一个RGB数据放入[39:16]
                    wr_cnt <= wr_cnt + 1'b1;
                end
                2'd1: begin
                    buffer[63:40] <= video_rgb_in_d0; // 第二个RGB数据放入[63:40]
                    buffer[15:14] <= 2'd2;            // 2表示所有字节都有效
                    buffer[13:8] <= 6'd0;             // 6个0
                    buffer[7:0] <= NODE_INFO;         // 节点信息
                    wr_cnt <= 2'd0;
                    // 拼接完成，输出数据
                    hdmi_axi_tx_valid <= 1'b1;
                    hdmi_axi_tx_data <= {video_rgb_in_d0, buffer[39:16], 2'd2, 6'd0, NODE_INFO};
                end
            endcase
        end else if(!video_de_in_d0 && prev_de) begin
            // DE下降沿处理（帧结束或行结束）
            if(wr_cnt == 2'd1) begin // 有未拼接完成的数据
                // 填充剩余部分并输出
                buffer[63:40] <= 24'd0;          // 第二个RGB位置填0
                buffer[15:14] <= 2'd1;            // 1表示只有第一个RGB数据有效
                wr_cnt <= 2'd0;
                // 输出不完整数据
                hdmi_axi_tx_valid <= 1'b1;
                hdmi_axi_tx_data <= {24'd0, buffer[39:16], 2'd1, 6'd0, NODE_INFO};      // 输出不完整数据，也结束。
            end else if(wr_cnt == 2'd0) begin
                // 行结束后，输出结束标志
                hdmi_axi_tx_valid <= 1'b1;
                hdmi_axi_tx_data <= {48'hf0f0f0f0f0f0, 2'd3, 6'd0, NODE_INFO};          // 输出结束标志
            end 
        end
    end
end
endmodule 