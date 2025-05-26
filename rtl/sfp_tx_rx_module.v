//----------------------------------------------------------------------------------------
// File name:           sfp_tx_module.v
// Created by:          珊瑚伊斯特
// Created date:        2025.5
// Version:             V0.1
// Descriptions:        SFP发送模块，包含环形队列和发送逻辑
//                      HDMI数据148.5MHz写入环形队列，156.25MHz读出发送到SFP
//----------------------------------------------------------------------------------------

module sfp_tx_rx_module(
    // 系统信号
    input                    sys_rst_n,           // 系统复位信号，低电平有效
    input                    locked_0,            // 锁相环锁定信号
    input                    wr_clk,              // 写时钟 148.5MHz
    input                    sys_clk,             // 系统时钟100MHz
    // input                    clk_ila,             // ILA时钟300MHz

    // HDMI数据输入接口（148.5MHz时钟域）
    input                 hdmi_data_valid,     // HDMI数据有效
    input      [63:0]     hdmi_data,           // HDMI数据

    // HDMI数据输出接口(分发给HDMI模块)
    output reg      hdmi_axi_rx_valid,      // HDMI接收数据有效
    output reg [63:0] hdmi_axi_rx_data,      // HDMI接收数据

    // SFP物理接口端口补全
    input           q0_ck1_n_in,            // 参考时钟负端156.25MHz
    input           q0_ck1_p_in,            // 参考时钟正端156.25MHz
    input           rxn_in,                 // SFP差分接收负端
    input           rxp_in,                 // SFP差分接收正端
    output          txn_out,                // SFP差分发送负端
    output          txp_out,                // SFP差分发送正端
    output          tx_disable,             // SFP发送使能
    output          tx_clk_out              // SFP发送时钟156.25MHz
);

//==========================================================================
// 参数定义
//==========================================================================
parameter LOCAL_NODE_INFO = 8'h12;      // 本地节点信息
localparam BOUNDARY_DATA = {48'hFEFEFEFEFEFE, 2'b11, 6'd0, LOCAL_NODE_INFO};
// 环形队列参数
localparam RING_DEPTH = 64;              // 64个深度的环形队列
localparam RING_ADDR_WIDTH = 6;          // 环形队列地址宽度为6位
// 数据类型定义
parameter DATA_TYPE_HDMI = 4'h2;    // HDMI数据类型 0010
parameter DATA_TYPE_ETH = 4'h1;     // 以太网数据类型 0001

//==========================================================================
// hdmi内部信号定义
//==========================================================================
// 环形队列存储
reg [63:0]      hdmi_ring_data[0:RING_DEPTH-1];  // 环形队列数据，64位宽
reg [RING_ADDR_WIDTH-1:0] hdmi_wr_ptr;           // 写指针（148.5MHz时钟域）
reg [RING_ADDR_WIDTH-1:0] hdmi_rd_ptr;           // 读指针（156.25MHz时钟域）

// 跨时钟域同步信号
(* ASYNC_REG = "TRUE" *) reg [RING_ADDR_WIDTH-1:0] hdmi_sync_wr_ptr_1;  // 写指针同步第一级
(* ASYNC_REG = "TRUE" *) reg [RING_ADDR_WIDTH-1:0] hdmi_sync_wr_ptr;    // 写指针同步第二级

// 队列状态信号
wire            hdmi_ring_empty;         // 环形队列空标志
wire            hdmi_ring_full;          // 环形队列满标志（用于写时钟域）
reg [RING_ADDR_WIDTH-1:0] hdmi_next_wr_ptr;   // 下一个写指针

// 发送控制信号
reg  [3:0]      send_type_cnt;           // 发送类型计数器

// 边界数据延时清零控制信号
reg  [3:0]      hdmi_boundary_delay_cnt;      // HDMI边界数据延时计数器（0-7，8个周期）
reg             hdmi_boundary_detected;       // HDMI边界数据检测标志
wire            hdmi_clear_all;               // 清零所有HDMI数据和指针的信号

// hdmi_clear_all跨时钟域同步（读时钟域 → 写时钟域）
(* ASYNC_REG = "TRUE" *) reg hdmi_clear_sync1;     // 清零信号同步第一级
(* ASYNC_REG = "TRUE" *) reg hdmi_clear_sync2;     // 清零信号同步第二级

//==========================================================================
// 收发器IP核内部信号定义
//==========================================================================
//SFP reg define
reg  [27:0] dely_500ms ;
reg  [2:0]  S_RESET;            //状态信号
reg  [15:0] cnt_rst;
reg         rst_n;
reg         gtwiz_reset_rx_datapath;
//SFP wire define
wire        stat_rx_status;
wire        tx_reset;
wire        rx_reset;
wire        gt0_rst;
wire        gtpowergood_out_0;        // GT电源状态信号
wire        tx_clk_out;             // 发送时钟
wire        gt_refclk_out;          // 参考时钟
wire        axis_to_sfp_tready;     // 发送使能
wire [55:0] rx_preambleout;  // 接收前导码
wire        rx_axis_tuser;
wire        tx_axis_tuser;

// AXI发送接口（发送到SFP）
reg         axis_to_sfp_tvalid;     // 数据有效信号
reg  [63:0] axis_to_sfp_tdata;      // 发送数据
reg         axis_to_sfp_tlast;      // 最后一拍标志
reg  [7:0]  axis_to_sfp_tkeep;      // 字节有效标志
// AXI接收接口（从SFP接收）
wire        sfp_to_axis_tvalid;     // 数据有效
wire [63:0] sfp_to_axis_tdata;      // 接收数据从SFP 
wire        sfp_to_axis_tlast;      // 最后一拍
wire [7:0]  sfp_to_axis_tkeep;      // 字节有效
// 复位控制
always@(posedge sys_clk)begin
    if(!locked_0)
        cnt_rst <= 16'b0;
    else if(!cnt_rst[15])
        cnt_rst <= cnt_rst + 1'b1;
    else
        cnt_rst <= cnt_rst;
end 

//SFP收发器复位信号，低电平有效
assign gt0_rst = ~rst_n;    // 系统复位时GT复位，高电平复位
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

//500ms计数器
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

//ILA实例化
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
//     // .probe12(video_de_posedge),   // video_de上升沿检测
//     // .probe13(video_de_negedge)    // video_de下降沿检测
//     // .probe12(test_valid),   // input wire [7:0]  probe6 
//     // .probe13(test_data)    // input wire [0:0]  probe5 
// );

//==========================================================================
// 环形队列状态判断
//==========================================================================
// 计算下一个写指针
always @(*) begin
    hdmi_next_wr_ptr = (hdmi_wr_ptr == RING_DEPTH-1) ? {RING_ADDR_WIDTH{1'b0}} : hdmi_wr_ptr + 1'b1;
end

// 环形队列满判断（写时钟域）
assign hdmi_ring_full = (hdmi_next_wr_ptr == hdmi_rd_ptr);

// 环形队列空判断（读时钟域）
assign hdmi_ring_empty = (hdmi_rd_ptr == hdmi_sync_wr_ptr);

//==========================================================================
// 环形队列写入逻辑（148.5MHz时钟域）
// 只允许在wr_clk时钟域对hdmi_ring_data赋值和清零，避免多驱动错误
//==========================================================================
integer i;  // 用于循环初始化的索引变量

always @(posedge wr_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        hdmi_wr_ptr <= {RING_ADDR_WIDTH{1'b0}};
        hdmi_clear_sync1 <= 1'b0;
        hdmi_clear_sync2 <= 1'b0;
        // 初始化环形队列
        for(i = 0; i < RING_DEPTH; i = i + 1) begin
            hdmi_ring_data[i] <= 64'd0;
        end
    end else begin
        // 同步hdmi_clear_all信号到写时钟域
        hdmi_clear_sync1 <= hdmi_clear_all;
        hdmi_clear_sync2 <= hdmi_clear_sync1;
        // 检测到清零信号时清零写指针和环形队列
        if (hdmi_clear_sync2) begin
            hdmi_wr_ptr <= {RING_ADDR_WIDTH{1'b0}};
            for(i = 0; i < RING_DEPTH; i = i + 1) begin
                hdmi_ring_data[i] <= 64'd0;
            end
        end else begin
            // 当有有效数据且队列不满时，写入数据
            if (hdmi_data_valid && !hdmi_ring_full) begin
                hdmi_ring_data[hdmi_wr_ptr] <= hdmi_data;           // 写入64位HDMI数据
                hdmi_wr_ptr <= hdmi_next_wr_ptr;                    // 移动写指针
            end
        end
    end
end

//==========================================================================
// 跨时钟域同步（156.25MHz时钟域）
//==========================================================================
always @(posedge tx_clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        hdmi_sync_wr_ptr_1 <= {RING_ADDR_WIDTH{1'b0}};
        hdmi_sync_wr_ptr <= {RING_ADDR_WIDTH{1'b0}};
    end else begin
        hdmi_sync_wr_ptr_1 <= hdmi_wr_ptr;                     // 第一级同步
        hdmi_sync_wr_ptr <= hdmi_sync_wr_ptr_1;                // 第二级同步
    end
end

//==========================================================================
// SFP发送逻辑（156.25MHz时钟域）
// 注意：此处不再对hdmi_ring_data数组进行任何赋值或清零操作！
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
        // 默认输出
        axis_to_sfp_tvalid <= 1'b0;
        axis_to_sfp_tdata  <= 64'h0;
        axis_to_sfp_tlast  <= 1'b0;
        axis_to_sfp_tkeep  <= 8'h0;

        // 边界数据延时清零逻辑
        if (hdmi_clear_all) begin
            // 只清零读指针和相关标志，不再清零hdmi_ring_data数组
            hdmi_rd_ptr <= {RING_ADDR_WIDTH{1'b0}};
            hdmi_boundary_detected <= 1'b0;      // 清除检测标志
            hdmi_boundary_delay_cnt <= 4'd0;     // 清零计数器
        end else if (hdmi_boundary_detected) begin
            // 边界数据检测后开始延时计数
            hdmi_boundary_delay_cnt <= hdmi_boundary_delay_cnt + 1'b1;
        end

        // 根据LOCAL_NODE_INFO低4位中1的位置轮询发送
        case (send_type_cnt)
            4'd0: begin  // 第0类数据，HDMI数据
                if (LOCAL_NODE_INFO[1]) begin  // 第1位对应HDMI数据
                    if (!hdmi_ring_empty && axis_to_sfp_tready) begin
                        // 直接输出环形队列数据
                        axis_to_sfp_tvalid <= 1'b1;
                        axis_to_sfp_tdata  <= hdmi_ring_data[hdmi_rd_ptr];
                        axis_to_sfp_tkeep  <= 8'hFF;
                        // 根据数据类型设置tlast
                        if ((hdmi_ring_data[hdmi_rd_ptr][15:14] == 2'd3) || (hdmi_ring_data[hdmi_rd_ptr][15:14] == 2'd1))
                            axis_to_sfp_tlast <= 1'b1;
                        else
                            axis_to_sfp_tlast <= 1'b0;
                        
                        // 检测是否为边界数据
                        if (hdmi_ring_data[hdmi_rd_ptr] == BOUNDARY_DATA) begin
                            hdmi_boundary_detected <= 1'b1;  // 设置边界检测标志
                            hdmi_boundary_delay_cnt <= 4'd0; // 开始计数
                        end
                        
                        // 读完后移动指针
                        hdmi_rd_ptr <= (hdmi_rd_ptr == RING_DEPTH-1) ? {RING_ADDR_WIDTH{1'b0}} : hdmi_rd_ptr + 1'b1;
                    end else begin
                        // 环形队列空时，tvalid为1，输出全0数据
                        axis_to_sfp_tvalid <= 1'b1;
                        axis_to_sfp_tdata  <= 64'h0;
                        axis_to_sfp_tlast  <= 1'b0;
                        axis_to_sfp_tkeep  <= 8'h00;
                    end
                end
                
                // 状态转换：根据LOCAL_NODE_INFO决定下一个状态
                if (LOCAL_NODE_INFO[0]) begin
                    send_type_cnt <= 4'd1;  // 下一个是第0类数据
                end else if (LOCAL_NODE_INFO[2]) begin
                    send_type_cnt <= 4'd2;  // 下一个是第2类数据
                end else if (LOCAL_NODE_INFO[3]) begin
                    send_type_cnt <= 4'd3;  // 下一个是第3类数据
                end else begin
                    send_type_cnt <= 4'd0;  // 只有第1类数据，保持在当前状态
                end
            end
            
            4'd1: begin  // 第1类数据（LOCAL_NODE_INFO[0]对应的数据类型）
                if (LOCAL_NODE_INFO[0]) begin
                    // 这里可以添加第0类数据的处理逻辑
                    // 当前只是占位，不发送数据
                end
                
                // 状态转换：跳转到下一个有效状态
                if (LOCAL_NODE_INFO[1]) begin
                    send_type_cnt <= 4'd0;  // 回到HDMI数据
                end else if (LOCAL_NODE_INFO[2]) begin
                    send_type_cnt <= 4'd2;  // 下一个是第2类数据
                end else if (LOCAL_NODE_INFO[3]) begin
                    send_type_cnt <= 4'd3;  // 下一个是第3类数据
                end else begin
                    send_type_cnt <= 4'd1;  // 只有第0类数据，保持在当前状态
                end
            end
            
            4'd2: begin  // 第2类数据（LOCAL_NODE_INFO[2]对应的数据类型）
                if (LOCAL_NODE_INFO[2]) begin
                    // 这里可以添加第2类数据的处理逻辑
                    // 当前只是占位，不发送数据
                end
                
                // 状态转换：跳转到下一个有效状态
                if (LOCAL_NODE_INFO[1]) begin
                    send_type_cnt <= 4'd0;  // 回到HDMI数据
                end else if (LOCAL_NODE_INFO[0]) begin
                    send_type_cnt <= 4'd1;  // 回到第0类数据
                end else if (LOCAL_NODE_INFO[3]) begin
                    send_type_cnt <= 4'd3;  // 下一个是第3类数据
                end else begin
                    send_type_cnt <= 4'd2;  // 只有第2类数据，保持在当前状态
                end
            end
            
            4'd3: begin  // 第3类数据（LOCAL_NODE_INFO[3]对应的数据类型）
                if (LOCAL_NODE_INFO[3]) begin
                    // 这里可以添加第3类数据的处理逻辑
                    // 当前只是占位，不发送数据
                end
                
                // 状态转换：跳转到下一个有效状态
                if (LOCAL_NODE_INFO[1]) begin
                    send_type_cnt <= 4'd0;  // 回到HDMI数据
                end else if (LOCAL_NODE_INFO[0]) begin
                    send_type_cnt <= 4'd1;  // 回到第0类数据
                end else if (LOCAL_NODE_INFO[2]) begin
                    send_type_cnt <= 4'd2;  // 回到第2类数据
                end else begin
                    send_type_cnt <= 4'd3;  // 只有第3类数据，保持在当前状态
                end
            end
            
            default: begin
                send_type_cnt <= 4'd0;
            end
        endcase
        
        // 当LOCAL_NODE_INFO全为0时，发送节点信息
        if (LOCAL_NODE_INFO[3:0] == 4'h0) begin
            axis_to_sfp_tvalid <= 1'b1;
            axis_to_sfp_tdata <= {56'h0, LOCAL_NODE_INFO[7:4], 4'h0};
            axis_to_sfp_tlast <= 1'b1;
            axis_to_sfp_tkeep <= 8'h01;
        end
    end
end
// 清零信号：当延时计数到8时触发
assign hdmi_clear_all = hdmi_boundary_detected && (hdmi_boundary_delay_cnt == 4'd7);

//==========================================================================
// SFP接收逻辑（156.25MHz时钟域）
//==========================================================================
always @(posedge tx_clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        // 复位时清零所有输出信号
        hdmi_axi_rx_valid <= 1'b0;
        hdmi_axi_rx_data <= 64'h0;
        // hdmi_axi_rx_keep <= 8'h0;
    end else begin
        if (sfp_to_axis_tvalid) begin
            // 检查数据类型字段，根据类型分发数据
            case (sfp_to_axis_tdata[3:0])  // 检查数据类型(高4位)
                DATA_TYPE_HDMI: begin  // HDMI数据类型
                    hdmi_axi_rx_valid <= 1'b1;
                    hdmi_axi_rx_data <= sfp_to_axis_tdata;
                    // hdmi_axi_rx_keep <= sfp_to_axis_tkeep;
                end
                DATA_TYPE_ETH: begin   // 以太网数据类型(预留)
                    // 暂时不处理以太网数据
                    hdmi_axi_rx_valid <= 1'b0;
                end
                default: begin         // 未知数据类型
                    hdmi_axi_rx_valid <= 1'b0;
                end
            endcase
        end else begin
            // 没有有效数据时，清零输出
            hdmi_axi_rx_valid <= 1'b0;
        end
    end
end

// 以太网核例化
xxv_ethernet_0 u_xxv_ethernet_0 (
  //光口差分数据
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

  //光口参考时钟
  .gt_refclk_p              (q0_ck1_p_in),  // input wire gt_refclk_p
  .gt_refclk_n              (q0_ck1_n_in),  // input wire gt_refclk_n
  
  .gt_refclk_out            (gt_refclk_out),    // output wire gt_refclk_out
  .gtpowergood_out_0        (gtpowergood_out_0),    // output wire gtpowergood_out_0
  .user_rx_reset_0          (rx_reset),  // output wire user_rx_reset_0
  
  //AXI4 Stream 发射接口信号
  .tx_axis_tready_0        (axis_to_sfp_tready),
  .tx_axis_tvalid_0        (axis_to_sfp_tvalid),
  .tx_axis_tdata_0         (axis_to_sfp_tdata),
  .tx_axis_tlast_0         (axis_to_sfp_tlast), 
  .tx_axis_tkeep_0         (axis_to_sfp_tkeep),
  .tx_axis_tuser_0         (1'b0),
  .tx_preamblein_0         (56'b0),            // input wire [55 : 0] tx_preamblein_0
  
  //RX 控制信号
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
  
  //RX 发送状态信号
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
  
  //AXI4 Stream 接收接口信号
  .rx_axis_tvalid_0        (sfp_to_axis_tvalid),
  .rx_axis_tdata_0         (sfp_to_axis_tdata), 
  .rx_axis_tlast_0         (sfp_to_axis_tlast),
  .rx_axis_tkeep_0         (sfp_to_axis_tkeep),
  .rx_axis_tuser_0         (1'b0),
  
  .tx_unfout_0                    ( ),                // output wire tx_unfout_0

  //TX 状态信号
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
  
  //AXI4?Stream 接口 - TX 路径控制信号和状态信号
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
assign  tx_disable = 2'b00;                     //打开光口
endmodule 