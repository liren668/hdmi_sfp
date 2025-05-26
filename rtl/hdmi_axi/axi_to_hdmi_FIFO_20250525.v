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

// FIFO参数
parameter  FIFO_WIDTH = 24;                         // FIFO宽度为24位
parameter  FIFO_DEPTH = 1024;                       // FIFO深度为1024

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
// 内部信号定义
//==========================================================================
// FIFO相关信号
reg              fifo_rst;                          // FIFO复位信号
reg [3:0]        fifo_rst_cnt;                      // FIFO复位计数器
reg              fifo_rd_en;                        // FIFO读使能
wire [FIFO_WIDTH-1:0] fifo_dout;                    // FIFO输出数据，24位
wire             fifo_empty;                        // FIFO空标志
reg [FIFO_WIDTH-1:0] buffer;                        // 写入数据缓冲，24位
reg              buf_valid;                         // 缓冲数据有效
reg              fifo_write_enable;                 // FIFO写入使能信号

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
    .probe8(fifo_dout),   // input wire [7:0]  probe6 
    .probe9(fifo_rd_en),   // input wire [7:0]  probe6 
    .probe10(video_de_posedge),   // input wire [7:0]  probe6 
    .probe11(video_de_negedge)   // input wire [7:0]  probe6 
    // .probe12(video_de_posedge),   // video_de上升沿检测
    // .probe13(video_de_negedge)    // video_de下降沿检测
    // .probe12(test_valid),   // input wire [7:0]  probe6 
    // .probe13(test_data)    // input wire [0:0]  probe5 
);

//==========================================================================
// AXI数据接收和写入FIFO (156.25MHz时钟域)
//==========================================================================
// FIFO实例化，24位宽，1024深度
fifo_generator_1 u_fifo_generator_1 (
    .rst           (fifo_rst),
    .wr_clk        (axi_clk),              // 写时钟为AXI时钟 156.25MHz
    .rd_clk        (pixel_clk),            // 读时钟为像素时钟 148.5MHz
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

// 数据接收和写入
always @(posedge axi_clk or negedge rst_n) begin
    if(!rst_n) begin
        fifo_rst <= 1'b1;                 // 复位FIFO
        fifo_rst_cnt <= 4'd8;             // 设置复位计数器为8个周期
        buffer <= {FIFO_WIDTH{1'b0}};     // 清零数据缓冲
        buf_valid <= 1'b0;
        pixel1_data <= 24'd0;
        pixel2_data <= 24'd0;
        pixel1_valid <= 1'b0;
        pixel2_valid <= 1'b0;
        boundary_detected <= 1'b0;
        boundary_ack_sync_1 <= 1'b0;
        boundary_ack_sync_2 <= 1'b0;
        vs_negedge_sync_1 <= 1'b0;        // 初始化VS下降沿同步寄存器
        vs_negedge_sync_2 <= 1'b0;        // 初始化VS下降沿同步寄存器
        vs_negedge_axi <= 1'b0;           // 初始化AXI域VS下降沿信号
        prev_video_de <= 1'b0;  // 初始化前一个周期视频使能信号
    end else begin
        // 复位FIFO逻辑
        if(fifo_rst) begin
            if(fifo_rst_cnt > 4'd0)
                fifo_rst_cnt <= fifo_rst_cnt - 4'd1;  // 计数器递减
            else begin
                fifo_rst <= 1'b0;            // 计数结束，释放复位信号
                fifo_rst_cnt <= 4'd8;        // 重置计数器，为下次复位准备
            end
        end
            
        // 同步VS下降沿到AXI时钟域
        vs_negedge_sync_1 <= video_vs_negedge;
        vs_negedge_sync_2 <= vs_negedge_sync_1;
        vs_negedge_axi <= vs_negedge_sync_2;  // AXI域的VS下降沿信号
            
        // 当检测到VS下降沿或video_de下降沿时，复位FIFO
        if((vs_negedge_axi || video_de_negedge) && !fifo_rst)
            fifo_rst <= 1'b1;  // 开始复位过程
        
        if (vs_negedge_axi) begin
            fifo_write_enable <= 1'b0;
        end
        
        // 默认情况下禁用写使能
        buf_valid <= 1'b0;
            
        // 边界确认信号的跨时钟域同步(pixel_clk -> axi_clk)
        boundary_ack_sync_1 <= boundary_ack;
        boundary_ack_sync_2 <= boundary_ack_sync_1;
        
        // 收到确认信号后清零边界检测标志
        if(boundary_ack_sync_2)
            boundary_detected <= 1'b0;
        
        // 处理有效的输入数据
        if(hdmi_axi_rx_valid) begin
            // 检查是否为边界数据
            if(hdmi_axi_rx_data[63:16] == VS_ON_BOUNDARY_DATA && hdmi_axi_rx_data[15:14] == BOUNDARY_FLAG) begin
                // 发现边界数据，设置边界检测标志 - 不写入FIFO
                boundary_detected <= 1'b1;
                fifo_write_enable <= 1'b1;
                // 清除像素有效标志
                pixel1_valid <= 1'b0;
                pixel2_valid <= 1'b0;
            end else begin
                // 非边界数据，提取像素并根据有效标志写入FIFO
                // 检查第一个像素是否有效（位[15:14]的对应位）
                if(hdmi_axi_rx_data[15:14] == 2'b01 || hdmi_axi_rx_data[15:14] == 2'b10) begin
                    pixel1_data <= hdmi_axi_rx_data[39:16];  // 存储第一个像素
                    pixel1_valid <= 1'b1;                    // 标记第一个像素有效
                    
                    // 写入第一个像素到FIFO
                    buffer <= hdmi_axi_rx_data[39:16];       // 写入第一个像素
                    buf_valid <= 1'b1;                       // 使能FIFO写入
                end
                
                // 检查第二个像素是否有效（双像素模式）
                if(hdmi_axi_rx_data[15:14] == 2'b10) begin
                    pixel2_data <= hdmi_axi_rx_data[63:40];  // 存储第二个像素
                    pixel2_valid <= 1'b1;                    // 标记第二个像素有效
                end else begin
                    pixel2_valid <= 1'b0;                    // 第二个像素无效
                end
            end
        end else if(pixel2_valid && fifo_write_enable) begin
            // 第二个像素有效，在下一个周期写入FIFO
            buffer <= pixel2_data;        // 写入第二个像素
            buf_valid <= 1'b1;            // 使能FIFO写入
            pixel2_valid <= 1'b0;         // 清除第二个像素有效标志
        end
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
        video_vs_prev <= 1'b0;        // 初始化前一个周期的VS状态
        video_vs_negedge <= 1'b0;     // 初始化VS下降沿检测
        video_rgb <= 24'd0;
        send_state <= 2'd0;
    end else begin
        // 默认禁用FIFO读使能
        fifo_rd_en <= 1'b0;
        
        // 更新前一周期的video_vs值
        video_vs_prev <= video_vs;
        
        // 检测video_vs的下降沿
        video_vs_negedge <= !video_vs && video_vs_prev;  // 下降沿：当前为0且前一周期为1
        
        // 更新前一周期的video_de值
        prev_video_de <= video_de;
        
        // 检测video_de的上升沿和下降沿
        video_de_posedge <= video_de && !prev_video_de;  // 上升沿：当前为1且前一周期为0
        video_de_negedge <= !video_de && prev_video_de;  // 下降沿：当前为0且前一周期为1
        
        // 在video_de上升沿时，预读第一个数据
        if(video_de_posedge && !fifo_empty) begin
            fifo_rd_en <= 1'b1;  // 使能FIFO读取
        end
        
        // 简化的数据读取逻辑
        if(video_de) begin
            video_rgb <= fifo_dout;  // 直接将FIFO数据输出到RGB
            fifo_rd_en <= 1'b1;      // 使能FIFO读取下一个数据
        end else begin
            // video_de无效时输出黑色
            video_rgb <= 24'd0;
            fifo_rd_en <= 1'b0;
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
        
       // 生成同步信号和数据使能 - 只有在sync_enable有效时才生成
        if(sync_enable) begin
            // 行同步信号生成
            video_hs <= (cnt_h < H_SYNC) ? 1'b0 : 1'b1;  //行同步信号赋值
            
            // 场同步信号生成
            video_vs <= (cnt_v < V_SYNC) ? 1'b0 : 1'b1;  //场同步信号赋值
            
            // 数据使能信号生成
            video_de <= ((cnt_h >= H_SYNC + H_BACK) && (cnt_h < H_SYNC + H_BACK + H_DISP)) && 
                         ((cnt_v >= V_SYNC + V_BACK) && (cnt_v < V_SYNC + V_BACK + V_DISP));
        end else begin
            // 未使能时关闭所有信号
            video_hs <= 1'b0;
            video_vs <= 1'b0;
            video_de <= 1'b0;
        end
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
endmodule