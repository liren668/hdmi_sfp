#时钟约束
create_clock -period 10.000 -name sys_clk_p -waveform {0.000 5.000}
create_clock -period 6.40 [get_ports q0_ck1_p_in]

# 允许时钟信号使用任意CMT列进行布�?
set_property CLOCK_DEDICATED_ROUTE ANY_CMT_COLUMN [get_nets u_clk_wiz_1/inst/clk_in1_clk_wiz_1]
# 将MMCM约束到合适的位置
#set_property LOC MMCM_X0Y3 [get_cells u_clk_wiz_1/inst/mmcme4_adv_inst]
# 将BUFGCE约束到合适的位置
#set_property LOC BUFGCE_X0Y95 [get_cells u_clk_wiz_1/inst/clkout1_buf]

#复位管脚
set_property -dict {PACKAGE_PIN F13 IOSTANDARD LVCMOS33} [get_ports sys_rst_n]

#时钟
set_property PACKAGE_PIN AE5 [get_ports sys_clk_p]
set_property PACKAGE_PIN AF5 [get_ports sys_clk_n]
set_property IOSTANDARD DIFF_HSTL_I_12 [get_ports sys_clk_p]
set_property IOSTANDARD DIFF_HSTL_I_12 [get_ports sys_clk_n]
set_property PACKAGE_PIN Y6 [get_ports q0_ck1_p_in]

#LED
set_property PACKAGE_PIN H13 [get_ports led]
set_property IOSTANDARD LVCMOS33 [get_ports led]

#SFPA
set_property -dict {PACKAGE_PIN B11 IOSTANDARD LVCMOS33} [get_ports {tx_disable[0]}]

#SFPB
set_property -dict {PACKAGE_PIN D11 IOSTANDARD LVCMOS33} [get_ports {tx_disable[1]}]

#------------------------HDMI_IN------------------------
set_property -dict {PACKAGE_PIN E10 IOSTANDARD LVCMOS33} [get_ports video_clk_in]
set_property -dict {PACKAGE_PIN J12 IOSTANDARD LVCMOS33} [get_ports video_vs_in]
set_property -dict {PACKAGE_PIN K12 IOSTANDARD LVCMOS33} [get_ports video_hs_in]
set_property -dict {PACKAGE_PIN K13 IOSTANDARD LVCMOS33} [get_ports video_de_in]
set_property -dict {PACKAGE_PIN AA10 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[0]}]
set_property -dict {PACKAGE_PIN AA11 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[1]}]
set_property -dict {PACKAGE_PIN Y10 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[2]}]
set_property -dict {PACKAGE_PIN W10 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[3]}]
set_property -dict {PACKAGE_PIN AA8 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[4]}]
set_property -dict {PACKAGE_PIN Y9 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[5]}]
set_property -dict {PACKAGE_PIN AB10 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[6]}]
set_property -dict {PACKAGE_PIN AB9 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[7]}]
set_property -dict {PACKAGE_PIN AD12 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[8]}]
set_property -dict {PACKAGE_PIN AC12 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[9]}]
set_property -dict {PACKAGE_PIN AC11 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[10]}]
set_property -dict {PACKAGE_PIN AB11 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[11]}]
set_property -dict {PACKAGE_PIN AF10 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[12]}]
set_property -dict {PACKAGE_PIN AE10 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[13]}]
set_property -dict {PACKAGE_PIN AD10 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[14]}]
set_property -dict {PACKAGE_PIN AD11 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[15]}]
set_property -dict {PACKAGE_PIN AG11 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[16]}]
set_property -dict {PACKAGE_PIN AF11 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[17]}]
set_property -dict {PACKAGE_PIN AH10 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[18]}]
set_property -dict {PACKAGE_PIN AG10 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[19]}]
set_property -dict {PACKAGE_PIN AH11 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[20]}]
set_property -dict {PACKAGE_PIN AH12 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[21]}]
set_property -dict {PACKAGE_PIN AF12 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[22]}]
set_property -dict {PACKAGE_PIN AE12 IOSTANDARD LVCMOS33} [get_ports {video_rgb_in[23]}]

#------------------------HDMI_OUT------------------------
set_property -dict {PACKAGE_PIN P9 IOSTANDARD LVCMOS18} [get_ports iic_scl]
set_property -dict {PACKAGE_PIN K5 IOSTANDARD LVCMOS18} [get_ports iic_sda]
set_property -dict {PACKAGE_PIN H12  IOSTANDARD LVCMOS33} [get_ports rst_hdmi_n]

set_property -dict {PACKAGE_PIN G11 IOSTANDARD LVCMOS33} [get_ports video_clk_out]
set_property -dict {PACKAGE_PIN G10 IOSTANDARD LVCMOS33} [get_ports video_vs_out]
set_property -dict {PACKAGE_PIN H11 IOSTANDARD LVCMOS33} [get_ports video_hs_out]
set_property -dict {PACKAGE_PIN J10 IOSTANDARD LVCMOS33} [get_ports video_de_out]
set_property -dict {PACKAGE_PIN W13 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[0]}]
set_property -dict {PACKAGE_PIN W14 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[1]}]
set_property -dict {PACKAGE_PIN Y14 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[2]}]
set_property -dict {PACKAGE_PIN Y13 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[3]}]
set_property -dict {PACKAGE_PIN AD14 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[4]}]
set_property -dict {PACKAGE_PIN AD15 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[5]}]
set_property -dict {PACKAGE_PIN AC13 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[6]}]
set_property -dict {PACKAGE_PIN AC14 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[7]}]
set_property -dict {PACKAGE_PIN W11 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[8]}]
set_property -dict {PACKAGE_PIN W12 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[9]}]
set_property -dict {PACKAGE_PIN AB13 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[10]}]
set_property -dict {PACKAGE_PIN AA13 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[11]}]
set_property -dict {PACKAGE_PIN Y12 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[12]}]
set_property -dict {PACKAGE_PIN AA12 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[13]}]
set_property -dict {PACKAGE_PIN AB14 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[14]}]
set_property -dict {PACKAGE_PIN AB15 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[15]}]
set_property -dict {PACKAGE_PIN AE15 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[16]}]
set_property -dict {PACKAGE_PIN AE14 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[17]}]
set_property -dict {PACKAGE_PIN AG14 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[18]}]
set_property -dict {PACKAGE_PIN AH14 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[19]}]
set_property -dict {PACKAGE_PIN AF13 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[20]}]
set_property -dict {PACKAGE_PIN AE13 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[21]}]
set_property -dict {PACKAGE_PIN AG13 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[22]}]
set_property -dict {PACKAGE_PIN AH13 IOSTANDARD LVCMOS33} [get_ports {video_rgb_out[23]}]