//----------------------------------------------------------------------------------------
// File name:           hdmi_top.v
// Created by:          珊瑚伊斯特
// Created date:        2025.5
// Version:             V0.1
// Descriptions:        HDMI顶层模块
//----------------------------------------------------------------------------------------

module hdmi_top (
    // 系统信号
    input                    rst_n,               // 复位信号，低电平有效
    input                    init_over,           // HDMI芯片初始化完成信号，高有效
    input                    axi_clk,             // AXI时钟 156.25MHz
    input                    clk_ila,             // ILA时钟 300MHz

    // HDMI输入信号
    input                    video_clk_in,        // 输入时钟148.5MHz
    input                    video_vs_in,         // 输入场同步信号
    input                    video_de_in,         // 数据使能
    input     [23:0]         video_rgb_in,        // RGB888颜色数据
    
    // HDMI输出信号
    input                    pixel_clk,          // 输出时钟148.5MHz
    output                   video_vs_out,        // 输出场同步信号
    output                   video_hs_out,        // 输出行同步信号
    output                   video_de_out,        // 输出数据使能
    output    [23:0]         video_rgb_out,       // 输出RGB888颜色数据
    
    // 输出到光口的AXI接口 (来自hdmi_to_axi模块)
    output                   hdmi_axi_tx_valid,   // 数据有效信号
    output     [63:0]        hdmi_axi_tx_data,    // 输出数据
    
    // 从光口输入的AXI接口
    input                    hdmi_axi_rx_valid,   // 输入数据有效信号
    input      [63:0]        hdmi_axi_rx_data    // 输入数据
);

//==========================================================================
// HDMI到AXI转换模块实例化
//==========================================================================
hdmi_to_axi u_hdmi_to_axi(
    // 系统信号
    .rst_n                  (rst_n              ),// 复位信号
    .init_over              (init_over          ),// HDMI芯片初始化完成信号，高有效
    // .clk_ila                (clk_ila            ),// ila调试时钟 300MHz

    // HDMI输入信号
    .video_clk_in           (video_clk_in       ),// 时钟148.5MHz
    .video_vs_in            (video_vs_in        ),// 输入场同步信号
    .video_de_in            (video_de_in        ),// 数据使能
    .video_rgb_in           (video_rgb_in       ),// RGB888颜色数据
    
    // 输出到AXI接口
    .hdmi_axi_tx_valid      (hdmi_axi_tx_valid  ),// 数据有效信号
    .hdmi_axi_tx_data       (hdmi_axi_tx_data   ) // 输出数据
);

//==========================================================================
// AXI到HDMI转换模块实例化
//==========================================================================
axi_to_hdmi u_axi_to_hdmi(
    // 系统信号
    .rst_n                  (rst_n              ),// 复位信号
    .axi_clk                (axi_clk             ),// AXI时钟 156.25MHz (与顶层rd_clk对应)
    .pixel_clk              (pixel_clk      ),// 像素时钟 148.5MHz (与顶层video_clk_out对应)
    .clk_ila                (clk_ila            ),// ila调试时钟 300MHz
    
    // AXI接口，从hdmi_to_axi接收数据
    .hdmi_axi_rx_valid      (hdmi_axi_rx_valid  ),// 输入数据有效信号
    .hdmi_axi_rx_data       (hdmi_axi_rx_data   ),// 输入数据
    
    // HDMI输出接口
    .video_hs               (video_hs_out       ),// 输出行同步信号
    .video_vs               (video_vs_out       ),// 输出场同步信号
    .video_de               (video_de_out       ),// 输出数据使能
    .video_rgb              (video_rgb_out      ) // 输出RGB888颜色数据
);

endmodule

