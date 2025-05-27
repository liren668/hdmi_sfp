//----------------------------------------------------------------------------------------
// File name:           ht_top.v
// Created by:          ɺ����˹��
// Modified by:         Claude
// Created date:        2025.4
// Version:             V0.3
// Descriptions:        HDMI-SFP��ϴ��䶥��ģ��
//----------------------------------------------------------------------------------------

module ht_top(
    // ϵͳ�ӿ�
    input           sys_clk_p,        // ϵͳʱ������100MHz
    input           sys_clk_n,        // ϵͳʱ�Ӹ���  
    input           sys_rst_n,        // ϵͳ��λ,�͵�ƽ��Ч
    //HDMIоƬ���ýӿ�
    output          rst_hdmi_n,       // HDMIоƬ��λ�źţ�����Ч
    output          iic_scl,          // HDMIоƬIICʱ��
    inout           iic_sda,          // HDMIоƬIIC����
    //HDMI����ӿ�
    input           video_clk_in,     // HDMIʱ��148.5MHz
    input           video_vs_in,      // ��ͬ���ź�
    input           video_de_in,      // ����ʹ��
    input    [23:0] video_rgb_in,     // RGB888��ɫ����
    //HDMI����ӿ�
    output          video_clk_out,    // HDMI���ʱ��148.5MHz
    output          video_vs_out,     // �����ͬ���ź�
    output          video_hs_out,     // �����ͬ���ź�
    output          video_de_out,     // �������ʹ��
    output   [23:0] video_rgb_out,    // ���RGB888��ɫ����
    // SFP�ӿ�
    input           q0_ck1_n_in,      // �ο�ʱ�Ӹ���156.25MHz
    input           q0_ck1_p_in,      // �ο�ʱ������
    input           rxn_in,           // ��ֽ��ո���
    input           rxp_in,           // ��ֽ�������
    output          txn_out,          // ��ַ��͸���
    output          txp_out,          // ��ַ�������
    output  [1:0]   tx_disable,          // ����ʹ��   

    //ָʾ��
    output         led                //ָʾ�� 
);

// HDMI���ݽӿ�
wire        hdmi_axi_tx_valid;    // ������Ч
wire [63:0] hdmi_axi_tx_data;     // HDMI���ݷ��͵�AXI�ӿ�

// HDMI��������ź�
wire        hdmi_axi_rx_valid;    // HDMI����������Ч
wire [63:0] hdmi_axi_rx_data;     // HDMI��������

//��ƵоƬ����ģ��
wire        hdmi_cfg_clk;     // ����ʱ��10Mhz
wire        rst_pll_lock;     
wire        init_over;        // HDMIоƬ��ʼ������ź�

//pll
wire        locked_0;
wire        locked_1;
wire        locked_2;
wire        video_clk_in_45deg;

wire        clk_ila; // ILAʱ��
wire        rst_cfg_hdmi; // HDMIоƬ��λ�ź�   

// SFPģ������źŶ���
wire        tx_clk_out;             // SFP����ʱ��156.25MHz
// wire        sfp_init_done;          // SFP��ʼ������ź�

//*****************************************************
//**                    main code
//*****************************************************

//��HDMI����ʱ���ȶ������߸�λ�ź�
assign  rst_pll_lock = sys_rst_n & locked_0; // sys_clk pll�����ȶ���ϵͳ��λʱHDMIоƬ��λ���ߵ�ƽ��λ 
assign rst_cfg_hdmi = sys_rst_n & locked_0; // HDMIоƬ��λ�ź� 
assign led = init_over;

// ϵͳʱ��
clk_wiz_0 u_clk_wiz_0(
    .clk_out1      (sys_clk),                // �ڲ�ʱ��100MHz
    .clk_out2      (hdmi_cfg_clk),           // ����ʱ��10Mhz
    .clk_out3      (clk_ila),                // ILAʱ��300MHz
    .reset         (~sys_rst_n),
    .locked        (locked_0),
    .clk_in1_p     (sys_clk_p),
    .clk_in1_n     (sys_clk_n)
);
//�ȶ�HDMI����ʱ��
clk_wiz_1 u_clk_wiz_1(
    .clk_out1               (video_clk_in_45deg),      //HDMI����ʱ��(150M)ƫ45��
    .reset                  (~sys_rst_n     ),
    .locked                 (locked_1       ),
    .clk_in1                (video_clk_in   )           //HDMI����ʱ��150M����Ҫȷ����148.5����150m
);
//HDMI���ʱ��
clk_wiz_2 u_clk_wiz_2(
    .clk_out1               (video_clk_out        ),      // HDMI���ʱ��150M
    .reset                  (~sys_rst_n      ),
    .locked                 (locked_2       ),
    .clk_in1                (sys_clk     )
);

//��ƵоƬ����ģ��
ms72xx_ctl u_ms72xx_ctl(
    .clk                    (hdmi_cfg_clk   ),      // ����ʱ��10Mhz
    .rst_n                  (rst_cfg_hdmi   ),
    
    .rstn_out               (rst_hdmi_n     ),      // HDMIоƬ��λ�źţ�����Ч
    .init_over              (init_over      ),
    .iic_scl                (iic_scl        ),
    .iic_sda                (iic_sda        )
);
// ʵ����HDMI����ģ��
hdmi_top u_hdmi_top(
    // ϵͳ�ź�
    .rst_n          (rst_pll_lock),           // ��λ�źţ��͵�ƽ��Ч
    .init_over      (init_over),              // HDMIоƬ��ʼ������źţ�����Ч
    .axi_clk        (tx_clk_out),             // AXIʱ��156.25MHz
    .clk_ila        (clk_ila),                // ILAʱ��300MHz
    
    // HDMI�����ź�
    .video_clk_in   (video_clk_in_45deg),     // ����ʱ��148.5MHz
    .video_vs_in    (video_vs_in),            // ���볡ͬ���ź�
    .video_de_in    (video_de_in),            // ����ʹ��
    .video_rgb_in   (video_rgb_in),           // RGB888��ɫ����
    
    // HDMI����ź�
    .pixel_clk      (video_clk_out),          // ���ʱ��148.5MHz
    .video_vs_out   (video_vs_out),           // �����ͬ���ź�
    .video_hs_out   (video_hs_out),           // �����ͬ���ź�
    .video_de_out   (video_de_out),           // �������ʹ��
    .video_rgb_out  (video_rgb_out),          // ���RGB888��ɫ����
    
    // �������ڵ�AXI�ӿ�
    .hdmi_axi_tx_valid    (hdmi_axi_tx_valid),  // ������Ч�ź�
    .hdmi_axi_tx_data     (hdmi_axi_tx_data),   // �������
    
    // �ӹ�������AXI�ӿ�
    .hdmi_axi_rx_valid    (hdmi_axi_rx_valid),  // ����������Ч�ź�
    .hdmi_axi_rx_data     (hdmi_axi_rx_data)    // ��������
);

// ʵ����SFP�շ�ģ��
sfp_tx_rx_module u_sfp_tx_rx_module(
    // ϵͳ�ź�
    .sys_rst_n          (sys_rst_n),           // ϵͳ��λ�źţ��͵�ƽ��Ч
    .locked_0           (locked_0),            // ���໷�����ź�
    .wr_clk             (video_clk_in_45deg),     // дʱ��148.5MHz
    .sys_clk            (sys_clk),                // ϵͳʱ��100MHz
    // .clk_ila            (clk_ila),                // ILAʱ��300MHz
    
    // HDMI��������ӿ�
    .hdmi_data_valid    (hdmi_axi_tx_valid),      // HDMI������Ч
    .hdmi_data          (hdmi_axi_tx_data),       // HDMI����
    
    // HDMI��������ӿ�
    .hdmi_axi_rx_valid  (hdmi_axi_rx_valid),      // HDMI����������Ч
    .hdmi_axi_rx_data   (hdmi_axi_rx_data),       // HDMI��������
    
    // SFP�ӿ�
    .q0_ck1_n_in        (q0_ck1_n_in),            // �ο�ʱ�Ӹ���156.25MHz
    .q0_ck1_p_in        (q0_ck1_p_in),            // �ο�ʱ������
    .rxn_in             (rxn_in),                 // ��ֽ��ո���
    .rxp_in             (rxp_in),                 // ��ֽ�������
    .txn_out            (txn_out),                // ��ַ��͸���
    .txp_out            (txp_out),                // ��ַ�������
    .tx_disable         (tx_disable),             // ����ʹ��
    // .sfp_init_done      (sfp_init_done),          // SFP��ʼ������ź�
    
    // ���ʱ��
    .tx_clk_out         (tx_clk_out)              // SFP����ʱ��156.25MHz
);

endmodule