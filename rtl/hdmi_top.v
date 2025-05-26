//----------------------------------------------------------------------------------------
// File name:           hdmi_top.v
// Created by:          ɺ����˹��
// Created date:        2025.5
// Version:             V0.1
// Descriptions:        HDMI����ģ��
//----------------------------------------------------------------------------------------

module hdmi_top (
    // ϵͳ�ź�
    input                    rst_n,               // ��λ�źţ��͵�ƽ��Ч
    input                    init_over,           // HDMIоƬ��ʼ������źţ�����Ч
    input                    axi_clk,             // AXIʱ�� 156.25MHz
    input                    clk_ila,             // ILAʱ�� 300MHz

    // HDMI�����ź�
    input                    video_clk_in,        // ����ʱ��148.5MHz
    input                    video_vs_in,         // ���볡ͬ���ź�
    input                    video_de_in,         // ����ʹ��
    input     [23:0]         video_rgb_in,        // RGB888��ɫ����
    
    // HDMI����ź�
    input                    pixel_clk,          // ���ʱ��148.5MHz
    output                   video_vs_out,        // �����ͬ���ź�
    output                   video_hs_out,        // �����ͬ���ź�
    output                   video_de_out,        // �������ʹ��
    output    [23:0]         video_rgb_out,       // ���RGB888��ɫ����
    
    // �������ڵ�AXI�ӿ� (����hdmi_to_axiģ��)
    output                   hdmi_axi_tx_valid,   // ������Ч�ź�
    output     [63:0]        hdmi_axi_tx_data,    // �������
    
    // �ӹ�������AXI�ӿ�
    input                    hdmi_axi_rx_valid,   // ����������Ч�ź�
    input      [63:0]        hdmi_axi_rx_data    // ��������
);

//==========================================================================
// HDMI��AXIת��ģ��ʵ����
//==========================================================================
hdmi_to_axi u_hdmi_to_axi(
    // ϵͳ�ź�
    .rst_n                  (rst_n              ),// ��λ�ź�
    .init_over              (init_over          ),// HDMIоƬ��ʼ������źţ�����Ч
    // .clk_ila                (clk_ila            ),// ila����ʱ�� 300MHz

    // HDMI�����ź�
    .video_clk_in           (video_clk_in       ),// ʱ��148.5MHz
    .video_vs_in            (video_vs_in        ),// ���볡ͬ���ź�
    .video_de_in            (video_de_in        ),// ����ʹ��
    .video_rgb_in           (video_rgb_in       ),// RGB888��ɫ����
    
    // �����AXI�ӿ�
    .hdmi_axi_tx_valid      (hdmi_axi_tx_valid  ),// ������Ч�ź�
    .hdmi_axi_tx_data       (hdmi_axi_tx_data   ) // �������
);

//==========================================================================
// AXI��HDMIת��ģ��ʵ����
//==========================================================================
axi_to_hdmi u_axi_to_hdmi(
    // ϵͳ�ź�
    .rst_n                  (rst_n              ),// ��λ�ź�
    .axi_clk                (axi_clk             ),// AXIʱ�� 156.25MHz (�붥��rd_clk��Ӧ)
    .pixel_clk              (pixel_clk      ),// ����ʱ�� 148.5MHz (�붥��video_clk_out��Ӧ)
    .clk_ila                (clk_ila            ),// ila����ʱ�� 300MHz
    
    // AXI�ӿڣ���hdmi_to_axi��������
    .hdmi_axi_rx_valid      (hdmi_axi_rx_valid  ),// ����������Ч�ź�
    .hdmi_axi_rx_data       (hdmi_axi_rx_data   ),// ��������
    
    // HDMI����ӿ�
    .video_hs               (video_hs_out       ),// �����ͬ���ź�
    .video_vs               (video_vs_out       ),// �����ͬ���ź�
    .video_de               (video_de_out       ),// �������ʹ��
    .video_rgb              (video_rgb_out      ) // ���RGB888��ɫ����
);

endmodule

