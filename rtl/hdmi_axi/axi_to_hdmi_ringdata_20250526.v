//----------------------------------------------------------------------------------------
// File name:           axi_to_hdmi.v
// Created by:          珊瑚伊斯特
// Created date:        2025/05/13 16:00:00
// Version:             V0.1
// Descriptions:        AXI到HDMI数据处理模块
//                      接收hdmi_to_axi的AXI数据，并将数据发送给HDMI显示接口
//----------------------------------------------------------------------------------------

module axi_to_hdmi(
    // 系统信号
    input                   rst_n,                  // 复位信号，低电平有效
    input                   axi_clk,                // AXI时钟 156.25MHz
    input                   pixel_clk,              // 像素时钟 148.5MHz
    input                   clk_ila,                // ila调试时钟 300MHz
    
    // AXI接口，从hdmi_to_axi接收数据
    input                   hdmi_axi_rx_valid,      // 数据有效信号
    input      [63:0]       hdmi_axi_rx_data,       // 输入数据
    
    // HDMI输出接口
    output reg          video_hs,               // 行同步信号
    output reg          video_vs,               // 场同步信号
    output reg          video_de,               // 数据使能
    output reg [23:0]   video_rgb               // RGB888颜色数据
);

//==========================================================================
// 参数定义
//==========================================================================
//启动延时
parameter  BOUNDARY_DELAY = 16;                      // 边界数据延时

// 边界数据定义
parameter  VS_ON_BOUNDARY_DATA = 48'hFEFEFEFEFEFE;  // VS置1特殊边界数据,vs_in信号。
parameter  BOUNDARY_FLAG = 2'b11;                   // 边界数据标志

// 时序参数
parameter  H_SYNC   =  12'd44;    // 行同步
parameter  H_BACK   =  12'd148;   // 行显示后沿
parameter  H_DISP   =  12'd1920;  // 行有效数据
parameter  H_FRONT  =  12'd88;    // 行显示前沿
parameter  H_TOTAL  =  12'd2200;  // 行扫描周期

parameter  V_SYNC   =  12'd5;     // 场同步
parameter  V_BACK   =  12'd36;    // 场显示后沿
parameter  V_DISP   =  12'd1080;  // 场有效数据
parameter  V_FRONT  =  12'd4;     // 场显示前沿
parameter  V_TOTAL  =  12'd1125;  // 场扫描周期

//==========================================================================
// 环形队列参数与信号定义
//==========================================================================
localparam RING_DEPTH = 128;              // 队列深度128
localparam RING_ADDR_WIDTH = 7;           // 地址宽度7位
reg [23:0] ring_data[0:RING_DEPTH-1];     // 24位宽，128深
reg [RING_ADDR_WIDTH-1:0] wr_ptr;         // 写指针（axi_clk域）
reg [RING_ADDR_WIDTH-1:0] rd_ptr;         // 读指针（pixel_clk域）
wire ring_full;
wire ring_empty;
reg [RING_ADDR_WIDTH-1:0] next_wr_ptr;
reg [RING_ADDR_WIDTH-1:0] wr_ptr_sync1, wr_ptr_sync2; // 写指针跨域同步到pixel_clk
reg [RING_ADDR_WIDTH-1:0] rd_ptr_sync1, rd_ptr_sync2; // 读指针跨域同步到axi_clk

// 数据接收寄存器
reg [23:0]       pixel1_data;                       // 第一个像素数据
reg [23:0]       pixel2_data;                       // 第二个像素数据
reg              pixel1_valid;                      // 第一个像素有效
reg              pixel2_valid;                      // 第二个像素有效

// 边界数据检测和同步
reg             boundary_detected;                 // AXI域边界检测标志
(* ASYNC_REG = "TRUE" *) reg  boundary_sync_1;     // 跨时钟域同步寄存器1
(* ASYNC_REG = "TRUE" *) reg  boundary_sync_2;     // 跨时钟域同步寄存器2
reg             boundary_pulse;                    // 同步脉冲
reg             boundary_ack;                      // 边界数据确认信号
(* ASYNC_REG = "TRUE" *) reg  boundary_ack_sync_1; // 确认信号同步寄存器1
(* ASYNC_REG = "TRUE" *) reg  boundary_ack_sync_2; // 确认信号同步寄存器2

// 计数器和控制信号
reg [11:0]      cnt_h;          // 行计数器
reg [11:0]      cnt_v;          // 场计数器
reg [7:0]       delay_cnt;      // 延时计数器
reg             sync_enable;    // 同步使能信号
integer         i;              // 循环计数器，用于初始化环形队列

// 在数据读取和处理部分，添加预读机制
reg             prev_video_de;         // 前一个周期的video_de状态

// 数据读取和输出逻辑
reg [1:0] send_state; // 0:初始状态 1:发送第一个数据 2:发送第二个数据

// 边沿检测信号
reg video_de_posedge;  // video_de上升沿检测
reg video_de_negedge;  // video_de下降沿检测
reg video_vs_prev;     // 前一个周期的video_vs状态
reg video_vs_negedge;  // video_vs下降沿检测
reg vs_negedge_sync_1; // VS下降沿同步寄存器1
reg vs_negedge_sync_2; // VS下降沿同步寄存器2
reg vs_negedge_axi;    // AXI时钟域的VS下降沿信号
(* ASYNC_REG = "TRUE" *) reg de_negedge_sync_1; // DE下降沿同步寄存器1
(* ASYNC_REG = "TRUE" *) reg de_negedge_sync_2; // DE下降沿同步寄存器2
reg de_negedge_axi;    // AXI时钟域的DE下降沿信号

// 添加状态机定义
localparam IDLE = 2'b00;      // 空闲状态
localparam WRITE_PIXEL1 = 2'b01;  // 写入第一个像素
localparam WRITE_PIXEL2 = 2'b10;  // 写入第二个像素

reg [1:0] write_state;  // 写入状态机
reg [63:0] pixel_data_reg;  // 临时存储像素数据

//ILA实例化
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
    // .probe12(video_de_posedge),   // video_de上升沿检测
    // .probe13(video_de_negedge)    // video_de下降沿检测
    // .probe12(test_valid),   // input wire [7:0]  probe6 
    // .probe13(test_data)    // input wire [0:0]  probe5 
);

//==========================================================================
// 队列状态判断
//==========================================================================
always @(*) begin
    next_wr_ptr = (wr_ptr == RING_DEPTH-1) ? 0 : wr_ptr + 1'b1;
end
assign ring_full = (next_wr_ptr == rd_ptr_sync2);
assign ring_empty = (rd_ptr == wr_ptr_sync2);

//==========================================================================
// 写指针跨时钟域同步到pixel_clk
//==========================================================================
always @(posedge pixel_clk or negedge rst_n) begin
    if (!rst_n) begin wr_ptr_sync1 <= 0; wr_ptr_sync2 <= 0; end
    else begin wr_ptr_sync1 <= wr_ptr; wr_ptr_sync2 <= wr_ptr_sync1; end
end
// 读指针跨时钟域同步到axi_clk
always @(posedge axi_clk or negedge rst_n) begin
    if (!rst_n) begin rd_ptr_sync1 <= 0; rd_ptr_sync2 <= 0; end
    else begin rd_ptr_sync1 <= rd_ptr; rd_ptr_sync2 <= rd_ptr_sync1; end
end

//==========================================================================
// 环形队列写入逻辑（AXI时钟域）156.25MHz
//==========================================================================
reg write_enable;
always @(posedge axi_clk or negedge rst_n) begin
    if(!rst_n) begin
        write_enable <= 1'b0;
        wr_ptr <= 0;
        boundary_detected <= 1'b0;  // 复位时清零
        write_state <= IDLE;
        pixel_data_reg <= 64'd0;
        for (i = 0; i < RING_DEPTH; i = i + 1)
            ring_data[i] <= 24'd0;
    end else begin
        case(write_state)
            IDLE: begin
                // 检查是否为边界数据
                if(hdmi_axi_rx_valid) begin
                    if(hdmi_axi_rx_data[63:16] == VS_ON_BOUNDARY_DATA && hdmi_axi_rx_data[15:14] == BOUNDARY_FLAG) begin
                        // 边界数据检测逻辑
                        boundary_detected <= 1'b1;
                        write_enable <= 1'b1;
                        pixel1_valid <= 1'b0;
                        pixel2_valid <= 1'b0;
                    end else if (write_enable) begin
                        // 非边界数据，准备写入像素到环形队列
                        case(hdmi_axi_rx_data[15:14])
                            2'b01: begin
                                ring_data[wr_ptr] <= hdmi_axi_rx_data[39:16];
                                wr_ptr <= (wr_ptr == RING_DEPTH-1) ? 0 : wr_ptr + 1'b1;
                            end
                            2'b10: begin
                                ring_data[wr_ptr] <= hdmi_axi_rx_data[39:16]; // 先写第一个像素
                                ring_data[(wr_ptr == RING_DEPTH-1) ? 0 : wr_ptr + 1'b1] <= hdmi_axi_rx_data[63:40]; // 再写第二个像素
                                wr_ptr <= (wr_ptr == RING_DEPTH-2) ? 0 : (wr_ptr == RING_DEPTH-1) ? 1 : wr_ptr + 2'b10;
                            end
                        endcase
                    end
                end
            end

            WRITE_PIXEL1: begin
                // 写入第一个像素
                ring_data[wr_ptr] <= pixel_data_reg[39:16];
                wr_ptr <= (wr_ptr == RING_DEPTH-1) ? 0 : wr_ptr + 1'b1;
                write_state <= WRITE_PIXEL2;  // 进入写入第二个像素状态
            end

            WRITE_PIXEL2: begin
                // 写入第二个像素
                ring_data[wr_ptr] <= pixel_data_reg[63:40];
                wr_ptr <= (wr_ptr == RING_DEPTH-1) ? 0 : wr_ptr + 1'b1;
                write_state <= IDLE;  // 返回空闲状态
            end
        endcase

        // 当边界确认信号同步回来时，清零边界检测标志
        if(boundary_ack_sync_2) begin
            boundary_detected <= 1'b0;  // 清零边界检测标志
        end

        // video_de下降沿或VS下降沿时清零写指针和队列
        if((vs_negedge_axi || de_negedge_axi)) begin
            wr_ptr <= 0;
            boundary_detected <= 1'b0;  // 同步信号到来时也清零
            write_state <= IDLE;  // 重置状态机
            for (i = 0; i < RING_DEPTH; i = i + 1)
                ring_data[i] <= 24'd0;
        end
        if(vs_negedge_axi) begin
            write_enable <= 1'b0;
            boundary_detected <= 1'b0;  // VS下降沿时也清零
            write_state <= IDLE;  // 重置状态机
        end
    end
end

//==========================================================================
// 环形队列读出逻辑（像素时钟域）148.5MHz
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
        // video_de下降沿或VS下降沿时清零读指针
        if (video_de_negedge || video_vs_negedge)
            rd_ptr <= 0;
    end
end

//==========================================================================
// 跨时钟域同步 (156.25MHz → 148.5MHz)
//==========================================================================
always @(posedge pixel_clk or negedge rst_n) begin
    if(!rst_n) begin
        boundary_sync_1 <= 1'b0;
        boundary_sync_2 <= 1'b0;
        boundary_pulse <= 1'b0;
        boundary_ack <= 1'b0;
    end else begin
        // 边界信号跨时钟域同步
        boundary_sync_1 <= boundary_detected;
        boundary_sync_2 <= boundary_sync_1;
        
        // 边界同步脉冲生成 - 上升沿检测
        boundary_pulse <= boundary_sync_1 && !boundary_sync_2;
        
        // 设置确认信号 - 当检测到边界脉冲时
        if(boundary_pulse)
            boundary_ack <= 1'b1;  // 发送确认信号
        else if(!boundary_sync_1)
            boundary_ack <= 1'b0;  // 当边界信号已经清零时，清除确认信号
    end
end

//==========================================================================
// 数据读取和处理 (148.5MHz时钟域)
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
        video_vs_prev <= 1'b0;        // 初始化前一个周期的VS状态
        video_vs_negedge <= 1'b0;     // 初始化VS下降沿检测
        video_rgb <= 24'd0;
        send_state <= 2'd0;
    end else begin
        
        // 更新前一周期的video_vs值
        video_vs_prev <= video_vs;
        
        // 检测video_vs的下降沿
        video_vs_negedge <= !video_vs && video_vs_prev;  // 下降沿：当前为0且前一周期为1
        
        // 更新前一周期的video_de值
        prev_video_de <= video_de0;
        
        // 检测video_de的上升沿和下降沿
        video_de_posedge <= video_de && !prev_video_de;  // 上升沿：当前为1且前一周期为0
        video_de_negedge <= !video_de && prev_video_de;  // 下降沿：当前为0且前一周期为1
        
        // 边界脉冲检测 - 从AXI域同步过来的边界数据
        if(boundary_pulse) begin
            if(!sync_enable) begin   // 如果同步未使能
                delay_cnt <= BOUNDARY_DELAY;  // 设置延时，默认为8
            end
        end
        // 延时计数处理
        if(!sync_enable && delay_cnt > 0) begin
            delay_cnt <= delay_cnt - 1'b1;
            if(delay_cnt == 8'd1) begin
                sync_enable <= 1'b1;     // 延时结束后启用同步信号生成
            end
        end
        // 本地计数器更新 - 受sync_enable控制
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
       // 生成同步信号和数据使能 - 只有在sync_enable有效时才生成
        if(sync_enable) begin
            // 行同步信号生成
            video_hs <= (cnt_h < H_SYNC) ? 1'b0 : 1'b1;  //行同步信号赋值
            
            // 场同步信号生成
            video_vs <= (cnt_v < V_SYNC) ? 1'b0 : 1'b1;  //场同步信号赋值
            
            // 数据使能信号生成
            video_de0 <= ((cnt_h >= H_SYNC + H_BACK) && (cnt_h < H_SYNC + H_BACK + H_DISP)) && 
                         ((cnt_v >= V_SYNC + V_BACK) && (cnt_v < V_SYNC + V_BACK + V_DISP));
        end else begin
            // 未使能时关闭所有信号
            video_hs <= 1'b0;
            video_vs <= 1'b0;
            video_de0 <= 1'b0;
        end
    end
end

//==========================================================================
// 增加丢帧检测：3帧无boundary_pulse则sync_enable清零
//==========================================================================
reg [1:0] no_boundary_vs_cnt; // 2位计数器，记录连续无boundary_pulse的VS周期数

// 在video_vs上升沿时判断是否有boundary_pulse
always @(posedge pixel_clk or negedge rst_n) begin
    if (!rst_n) begin
        no_boundary_vs_cnt <= 2'd0;
    end else begin
        // video_vs上升沿检测
        if (video_vs && !video_vs_prev) begin
            if (!boundary_pulse) begin
                if (no_boundary_vs_cnt < 2'd3)
                    no_boundary_vs_cnt <= no_boundary_vs_cnt + 1'b1;
            end else begin
                no_boundary_vs_cnt <= 2'd0; // 检测到boundary_pulse则清零
            end
        end
        // 只要检测到boundary_pulse，随时清零计数器
        if (boundary_pulse)
            no_boundary_vs_cnt <= 2'd0;
    end
end

// 当连续3帧无boundary_pulse时，sync_enable自动清零
always @(posedge pixel_clk or negedge rst_n) begin
    if (!rst_n) begin
        sync_enable <= 1'b0;
    end else if (no_boundary_vs_cnt == 2'd3) begin
        sync_enable <= 1'b0;
    end
end

//==========================================================================
// 跨时钟域同步 (148.5MHz → 156.25MHz)
//==========================================================================
always @(posedge axi_clk or negedge rst_n) begin
    if(!rst_n) begin
        vs_negedge_sync_1 <= 1'b0;
        vs_negedge_sync_2 <= 1'b0;
        vs_negedge_axi <= 1'b0;
    end else begin
        // 同步VS下降沿到AXI时钟域
        vs_negedge_sync_1 <= video_vs_negedge;
        vs_negedge_sync_2 <= vs_negedge_sync_1;
        vs_negedge_axi <= vs_negedge_sync_2;
    end
end

//==========================================================================
// DE下降沿跨时钟域同步 (148.5MHz → 156.25MHz)
//==========================================================================
always @(posedge axi_clk or negedge rst_n) begin
    if(!rst_n) begin
        de_negedge_sync_1 <= 1'b0;
        de_negedge_sync_2 <= 1'b0;
        de_negedge_axi <= 1'b0;
    end else begin
        // 同步DE下降沿到AXI时钟域
        de_negedge_sync_1 <= video_de_negedge;
        de_negedge_sync_2 <= de_negedge_sync_1;
        de_negedge_axi <= de_negedge_sync_2;
    end
end
endmodule