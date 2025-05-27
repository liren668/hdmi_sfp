//----------------------------------------------------------------------------------------
// File name:           ht_top.v
// Created by:          珊瑚伊斯特
// Modified by:         Claude
// Created date:        2025.4
// Version:             V0.3
// Descriptions:        HDMI-SFP混合传输顶层模块
//----------------------------------------------------------------------------------------

module ht_top(
    // 系统接口
    input           sys_clk_p,        // 系统时钟正端100MHz
    input           sys_clk_n,        // 系统时钟负端  
    input           sys_rst_n,        // 系统复位,低电平有效
    //HDMI芯片配置接口
    output          rst_hdmi_n,       // HDMI芯片复位信号，低有效
    output          iic_scl,          // HDMI芯片IIC时钟
    inout           iic_sda,          // HDMI芯片IIC数据
    //HDMI输入接口
    input           video_clk_in,     // HDMI时钟148.5MHz
    input           video_vs_in,      // 场同步信号
    input           video_de_in,      // 数据使能
    input    [23:0] video_rgb_in,     // RGB888颜色数据
    //HDMI输出接口
    output          video_clk_out,    // HDMI输出时钟148.5MHz
    output          video_vs_out,     // 输出场同步信号
    output          video_hs_out,     // 输出行同步信号
    output          video_de_out,     // 输出数据使能
    output   [23:0] video_rgb_out,    // 输出RGB888颜色数据
    // SFP接口
    input           q0_ck1_n_in,      // 参考时钟负端156.25MHz
    input           q0_ck1_p_in,      // 参考时钟正端
    input           rxn_in,           // 差分接收负端
    input           rxp_in,           // 差分接收正端
    output          txn_out,          // 差分发送负端
    output          txp_out,          // 差分发送正端
    output  [1:0]   tx_disable,          // 发送使能   

    //指示灯
    output         led                //指示灯 
);

// HDMI数据接口
wire        hdmi_axi_tx_valid;    // 数据有效
wire [63:0] hdmi_axi_tx_data;     // HDMI数据发送到AXI接口

// HDMI接收相关信号
wire        hdmi_axi_rx_valid;    // HDMI接收数据有效
wire [63:0] hdmi_axi_rx_data;     // HDMI接收数据

//视频芯片控制模块
wire        hdmi_cfg_clk;     // 配置时钟10Mhz
wire        rst_pll_lock;     
wire        init_over;        // HDMI芯片初始化完成信号

//pll
wire        locked_0;
wire        locked_1;
wire        locked_2;
wire        video_clk_in_45deg;

wire        clk_ila; // ILA时钟
wire        rst_cfg_hdmi; // HDMI芯片复位信号   

// SFP模块相关信号定义
wire        tx_clk_out;             // SFP发送时钟156.25MHz
// wire        sfp_init_done;          // SFP初始化完成信号

//*****************************************************
//**                    main code
//*****************************************************

//待HDMI配置时钟稳定后拉高复位信号
assign  rst_pll_lock = sys_rst_n & locked_0; // sys_clk pll锁相稳定后，系统复位时HDMI芯片复位，高电平复位 
assign rst_cfg_hdmi = sys_rst_n & locked_0; // HDMI芯片复位信号 
assign led = init_over;

// 系统时钟
clk_wiz_0 u_clk_wiz_0(
    .clk_out1      (sys_clk),                // 内部时钟100MHz
    .clk_out2      (hdmi_cfg_clk),           // 配置时钟10Mhz
    .clk_out3      (clk_ila),                // ILA时钟300MHz
    .reset         (~sys_rst_n),
    .locked        (locked_0),
    .clk_in1_p     (sys_clk_p),
    .clk_in1_n     (sys_clk_n)
);
//稳定HDMI输入时钟
clk_wiz_1 u_clk_wiz_1(
    .clk_out1               (video_clk_in_45deg),      //HDMI输入时钟(150M)偏45°
    .reset                  (~sys_rst_n     ),
    .locked                 (locked_1       ),
    .clk_in1                (video_clk_in   )           //HDMI输入时钟150M，需要确认是148.5还是150m
);
//HDMI输出时钟
clk_wiz_2 u_clk_wiz_2(
    .clk_out1               (video_clk_out        ),      // HDMI输出时钟150M
    .reset                  (~sys_rst_n      ),
    .locked                 (locked_2       ),
    .clk_in1                (sys_clk     )
);

//视频芯片控制模块
ms72xx_ctl u_ms72xx_ctl(
    .clk                    (hdmi_cfg_clk   ),      // 配置时钟10Mhz
    .rst_n                  (rst_cfg_hdmi   ),
    
    .rstn_out               (rst_hdmi_n     ),      // HDMI芯片复位信号，低有效
    .init_over              (init_over      ),
    .iic_scl                (iic_scl        ),
    .iic_sda                (iic_sda        )
);
// 实例化HDMI顶层模块
hdmi_top u_hdmi_top(
    // 系统信号
    .rst_n          (rst_pll_lock),           // 复位信号，低电平有效
    .init_over      (init_over),              // HDMI芯片初始化完成信号，高有效
    .axi_clk        (tx_clk_out),             // AXI时钟156.25MHz
    .clk_ila        (clk_ila),                // ILA时钟300MHz
    
    // HDMI输入信号
    .video_clk_in   (video_clk_in_45deg),     // 输入时钟148.5MHz
    .video_vs_in    (video_vs_in),            // 输入场同步信号
    .video_de_in    (video_de_in),            // 数据使能
    .video_rgb_in   (video_rgb_in),           // RGB888颜色数据
    
    // HDMI输出信号
    .pixel_clk      (video_clk_out),          // 输出时钟148.5MHz
    .video_vs_out   (video_vs_out),           // 输出场同步信号
    .video_hs_out   (video_hs_out),           // 输出行同步信号
    .video_de_out   (video_de_out),           // 输出数据使能
    .video_rgb_out  (video_rgb_out),          // 输出RGB888颜色数据
    
    // 输出到光口的AXI接口
    .hdmi_axi_tx_valid    (hdmi_axi_tx_valid),  // 数据有效信号
    .hdmi_axi_tx_data     (hdmi_axi_tx_data),   // 输出数据
    
    // 从光口输入的AXI接口
    .hdmi_axi_rx_valid    (hdmi_axi_rx_valid),  // 输入数据有效信号
    .hdmi_axi_rx_data     (hdmi_axi_rx_data)    // 输入数据
);

// 实例化SFP收发模块
sfp_tx_rx_module u_sfp_tx_rx_module(
    // 系统信号
    .sys_rst_n          (sys_rst_n),           // 系统复位信号，低电平有效
    .locked_0           (locked_0),            // 锁相环锁定信号
    .wr_clk             (video_clk_in_45deg),     // 写时钟148.5MHz
    .sys_clk            (sys_clk),                // 系统时钟100MHz
    // .clk_ila            (clk_ila),                // ILA时钟300MHz
    
    // HDMI数据输入接口
    .hdmi_data_valid    (hdmi_axi_tx_valid),      // HDMI数据有效
    .hdmi_data          (hdmi_axi_tx_data),       // HDMI数据
    
    // HDMI数据输出接口
    .hdmi_axi_rx_valid  (hdmi_axi_rx_valid),      // HDMI接收数据有效
    .hdmi_axi_rx_data   (hdmi_axi_rx_data),       // HDMI接收数据
    
    // SFP接口
    .q0_ck1_n_in        (q0_ck1_n_in),            // 参考时钟负端156.25MHz
    .q0_ck1_p_in        (q0_ck1_p_in),            // 参考时钟正端
    .rxn_in             (rxn_in),                 // 差分接收负端
    .rxp_in             (rxp_in),                 // 差分接收正端
    .txn_out            (txn_out),                // 差分发送负端
    .txp_out            (txp_out),                // 差分发送正端
    .tx_disable         (tx_disable),             // 发送使能
    // .sfp_init_done      (sfp_init_done),          // SFP初始化完成信号
    
    // 输出时钟
    .tx_clk_out         (tx_clk_out)              // SFP发送时钟156.25MHz
);

endmodule