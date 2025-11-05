#######################################################
#                                                     
#  Innovus Command Logging File                     
#  Created on Fri Oct 31 12:48:08 2025                
#                                                     
#######################################################

#@(#)CDS: Innovus v20.14-s095_1 (64bit) 04/19/2021 14:41 (Linux 2.6.32-431.11.2.el6.x86_64)
#@(#)CDS: NanoRoute 20.14-s095_1 NR210411-1939/20_14-UB (database version 18.20.547) {superthreading v2.13}
#@(#)CDS: AAE 20.14-s018 (64bit) 04/19/2021 (Linux 2.6.32-431.11.2.el6.x86_64)
#@(#)CDS: CTE 20.14-s027_1 () Apr 13 2021 21:29:07 ( )
#@(#)CDS: SYNTECH 20.14-s017_1 () Mar 25 2021 13:07:27 ( )
#@(#)CDS: CPE v20.14-s080
#@(#)CDS: IQuantus/TQuantus 20.1.1-s460 (64bit) Fri Mar 5 18:46:16 PST 2021 (Linux 2.6.32-431.11.2.el6.x86_64)

set_global _enable_mmmc_by_default_flow      $CTE::mmmc_default
suppressMessage ENCEXT-2799
getVersion
getVersion
getVersion
win
set init_gnd_net VSS
set init_lef_file ../../install/FOUNDRY/digital/90nm/dig/lef/gsclib090_translated.lef
set init_verilog cache_memory_90nm_netlist.v
set init_mmmc_file Default.view
set init_pwr_net VDD
init_design
getIoFlowFlag
setIoFlowFlag 0
floorPlan -site gsclib090site -r 1 0.7 6 6 6 6
uiSetTool select
getIoFlowFlag
fit
clearGlobalNets
globalNetConnect VDD -type pgpin -pin VDD -instanceBasename * -hierarchicalInstance {}
globalNetConnect VSS -type pgpin -pin VSS -instanceBasename * -hierarchicalInstance {}
clearGlobalNets
globalNetConnect VDD -type pgpin -pin VDD -instanceBasename * -hierarchicalInstance {}
globalNetConnect VSS -type pgpin -pin VSS -instanceBasename * -hierarchicalInstance {}
clearGlobalNets
globalNetConnect VDD -type pgpin -pin VDD -instanceBasename * -hierarchicalInstance {}
globalNetConnect VSS -type pgpin -pin VSS -instanceBasename * -hierarchicalInstance {}
set sprCreateIeRingOffset 1.0
set sprCreateIeRingThreshold 1.0
set sprCreateIeRingJogDistance 1.0
set sprCreateIeRingLayers {}
set sprCreateIeRingOffset 1.0
set sprCreateIeRingThreshold 1.0
set sprCreateIeRingJogDistance 1.0
set sprCreateIeRingLayers {}
set sprCreateIeStripeWidth 10.0
set sprCreateIeStripeThreshold 1.0
set sprCreateIeStripeWidth 10.0
set sprCreateIeStripeThreshold 1.0
set sprCreateIeRingOffset 1.0
set sprCreateIeRingThreshold 1.0
set sprCreateIeRingJogDistance 1.0
set sprCreateIeRingLayers {}
set sprCreateIeStripeWidth 10.0
set sprCreateIeStripeThreshold 1.0
setAddRingMode -ring_target default -extend_over_row 0 -ignore_rows 0 -avoid_short 0 -skip_crossing_trunks none -stacked_via_top_layer Metal9 -stacked_via_bottom_layer Metal1 -via_using_exact_crossover_size 1 -orthogonal_only true -skip_via_on_pin {  standardcell } -skip_via_on_wire_shape {  noshape }
addRing -nets {VDD VSS} -type core_rings -follow core -layer {top Metal9 bottom Metal9 left Metal8 right Metal8} -width {top 1.8 bottom 1.8 left 1.8 right 1.8} -spacing {top 0.5 bottom 0.5 left 0.5 right 0.5} -offset {top 1.8 bottom 1.8 left 1.8 right 1.8} -center 1 -threshold 0 -jog_distance 0 -snap_wire_center_to_grid None
set sprCreateIeRingOffset 1.0
set sprCreateIeRingThreshold 1.0
set sprCreateIeRingJogDistance 1.0
set sprCreateIeRingLayers {}
set sprCreateIeRingOffset 1.0
set sprCreateIeRingThreshold 1.0
set sprCreateIeRingJogDistance 1.0
set sprCreateIeRingLayers {}
set sprCreateIeStripeWidth 10.0
set sprCreateIeStripeThreshold 1.0
set sprCreateIeStripeWidth 10.0
set sprCreateIeStripeThreshold 1.0
set sprCreateIeRingOffset 1.0
set sprCreateIeRingThreshold 1.0
set sprCreateIeRingJogDistance 1.0
set sprCreateIeRingLayers {}
set sprCreateIeStripeWidth 10.0
set sprCreateIeStripeThreshold 1.0
setAddStripeMode -ignore_block_check false -break_at none -route_over_rows_only false -rows_without_stripes_only false -extend_to_closest_target none -stop_at_last_wire_for_area false -partial_set_thru_domain false -ignore_nondefault_domains false -trim_antenna_back_to_shape none -spacing_type edge_to_edge -spacing_from_block 0 -stripe_min_length stripe_width -stacked_via_top_layer Metal9 -stacked_via_bottom_layer Metal1 -via_using_exact_crossover_size false -split_vias false -orthogonal_only true -allow_jog { padcore_ring  block_ring } -skip_via_on_pin {  standardcell } -skip_via_on_wire_shape {  noshape   }
addStripe -nets {} -layer Metal9 -direction horizontal -width 1.8 -spacing 0.5 -number_of_sets 5 -start_from bottom -switch_layer_over_obs false -max_same_layer_jog_length 2 -padcore_ring_top_layer_limit Metal9 -padcore_ring_bottom_layer_limit Metal1 -block_ring_top_layer_limit Metal9 -block_ring_bottom_layer_limit Metal1 -use_wire_group 0 -snap_wire_center_to_grid None
zoomBox 140.33750 107.81050 766.69750 410.63300
zoomBox -318.66050 -1.74500 1093.00050 680.74150
zoomBox -464.26550 -36.49850 1196.51200 766.42700
setAddStripeMode -ignore_block_check false -break_at none -route_over_rows_only false -rows_without_stripes_only false -extend_to_closest_target none -stop_at_last_wire_for_area false -partial_set_thru_domain false -ignore_nondefault_domains false -trim_antenna_back_to_shape none -spacing_type edge_to_edge -spacing_from_block 0 -stripe_min_length stripe_width -stacked_via_top_layer Metal9 -stacked_via_bottom_layer Metal1 -via_using_exact_crossover_size false -split_vias false -orthogonal_only true -allow_jog { padcore_ring  block_ring } -skip_via_on_pin {  standardcell } -skip_via_on_wire_shape {  noshape   }
addStripe -nets {VSS VDD} -layer Metal8 -direction horizontal -width 1.8 -spacing 0.5 -number_of_sets 5 -start_from bottom -switch_layer_over_obs false -max_same_layer_jog_length 2 -padcore_ring_top_layer_limit Metal9 -padcore_ring_bottom_layer_limit Metal1 -block_ring_top_layer_limit Metal9 -block_ring_bottom_layer_limit Metal1 -use_wire_group 0 -snap_wire_center_to_grid None
setAddStripeMode -ignore_block_check false -break_at none -route_over_rows_only false -rows_without_stripes_only false -extend_to_closest_target none -stop_at_last_wire_for_area false -partial_set_thru_domain false -ignore_nondefault_domains false -trim_antenna_back_to_shape none -spacing_type edge_to_edge -spacing_from_block 0 -stripe_min_length stripe_width -stacked_via_top_layer Metal9 -stacked_via_bottom_layer Metal1 -via_using_exact_crossover_size false -split_vias false -orthogonal_only true -allow_jog { padcore_ring  block_ring } -skip_via_on_pin {  standardcell } -skip_via_on_wire_shape {  noshape   }
addStripe -nets {VSS VDD} -layer Metal9 -direction vertical -width 1.8 -spacing 0.5 -number_of_sets 5 -start_from left -switch_layer_over_obs false -max_same_layer_jog_length 2 -padcore_ring_top_layer_limit Metal9 -padcore_ring_bottom_layer_limit Metal1 -block_ring_top_layer_limit Metal9 -block_ring_bottom_layer_limit Metal1 -use_wire_group 0 -snap_wire_center_to_grid None
setPlaceMode -fp false
place_design
zoomBox -257.54200 -26.31200 1154.11850 656.17450
zoomBox -80.42650 -14.97950 1119.48500 565.13400
zoomBox 300.37350 120.02800 926.73600 422.85150
zoomBox 477.52750 197.80250 804.49300 355.87850
zoomBox 585.11750 245.01800 730.19450 315.15750
zoomBox 632.82200 265.93400 697.19400 297.05550
zoomBox 647.51950 272.36750 687.05200 291.48000
zoomBox 618.19100 259.55150 707.28900 302.62700
zoomBox 531.12650 221.50500 767.36650 335.71850
zoomBox 300.27600 120.62500 926.66200 423.46000
zoomBox -311.81550 -146.85450 1349.03050 656.10400
zoomBox -1934.75550 -856.06650 2468.92600 1272.95350
zoomBox -689.26500 -311.79700 1609.48600 799.56550
zoomBox -164.40800 -82.43850 1247.31300 600.07700
zoomBox 234.86850 92.04150 971.79600 448.31900
zoomBox 443.29250 183.12150 827.97400 369.10100
zoomBox 570.04200 238.19600 740.72800 320.71650
zoomBox 626.28100 262.63300 702.01700 299.24850
zoomBox 575.06400 247.70150 745.75400 330.22400
zoomBox 490.66250 222.88850 817.65400 380.97700
zoomBox 328.97800 175.35550 955.39100 478.20350
zoomBox 244.43300 123.33900 1111.44050 542.50550
zoomBox 181.15650 32.22850 1381.16700 612.39000
gui_select -rect {396.91300 363.44000 622.77200 364.16150}
deselectAll
zoomBox -815.81950 -402.02700 1888.70250 905.51100
fit
setOptMode -fixCap true -fixTran true -fixFanoutLoad false
optDesign -preCTS
zoomBox -332.88800 -0.09600 1078.76900 682.38900
setSrouteMode -viaConnectToShape { noshape }
sroute -connect { blockPin padPin padRing corePin floatingStripe } -layerChangeRange { Metal1(1) Metal9(9) } -blockPinTarget { nearestTarget } -padPinPortConnect { allPort oneGeom } -padPinTarget { nearestTarget } -corePinTarget { firstAfterRowEnd } -floatingStripeTarget { blockring padring ring stripe ringpin blockpin followpin } -allowJogging 1 -crossoverViaLayerRange { Metal1(1) Metal9(9) } -nets { VDD VSS } -allowLayerChange 1 -blockPin useLef -targetViaLayerRange { Metal1(1) Metal9(9) }
setNanoRouteMode -quiet -timingEngine {}
setNanoRouteMode -quiet -routeWithTimingDriven 1
setNanoRouteMode -quiet -routeWithSiDriven 1
setNanoRouteMode -quiet -routeTopRoutingLayer 9
setNanoRouteMode -quiet -routeBottomRoutingLayer 1
setNanoRouteMode -quiet -drouteEndIteration 1
setNanoRouteMode -quiet -routeWithTimingDriven true
setNanoRouteMode -quiet -routeWithSiDriven true
routeDesign -globalDetail
zoomBox -31.86600 162.04300 835.06900 581.17450
zoomBox 107.19800 228.11050 733.55900 530.93350
zoomBox 312.12150 320.46350 590.04200 454.82800
zoomBox 406.70100 341.39200 551.77850 411.53150
zoomBox 464.25600 354.05800 528.62900 385.18000
zoomBox 422.21750 344.80650 545.53700 404.42700
zoomBox 341.68450 327.08400 577.92650 441.29850
zoomBox 187.53950 293.26300 640.10500 512.06200
zoomBox -107.75450 228.47250 759.21900 647.62250
zoomBox -673.44550 104.35500 987.40450 907.31550
zoomBox -1757.13050 -133.41550 1424.53650 1404.80450
zoomBox -2627.90550 -324.47200 1775.78600 1804.55250
fit
setLayerPreference node_layer -isSelectable 0
setLayerPreference node_layer -isSelectable 1
setLayerPreference node_layer -isVisible 0
setLayerPreference node_layer -isVisible 1
setLayerPreference node_layer -isSelectable 0
setLayerPreference node_layer -isSelectable 1
redirect -quiet {set honorDomain [getAnalysisMode -honorClockDomains]} > /dev/null
timeDesign -preCTS -pathReports -drvReports -slackReports -numPaths 50 -prefix cache_memory_preCTS -outDir timingReports
getMultiCpuUsage -localCpu
get_verify_drc_mode -disable_rules -quiet
get_verify_drc_mode -quiet -area
get_verify_drc_mode -quiet -layer_range
get_verify_drc_mode -check_ndr_spacing -quiet
get_verify_drc_mode -check_only -quiet
get_verify_drc_mode -check_same_via_cell -quiet
get_verify_drc_mode -exclude_pg_net -quiet
get_verify_drc_mode -ignore_trial_route -quiet
get_verify_drc_mode -max_wrong_way_halo -quiet
get_verify_drc_mode -use_min_spacing_on_block_obs -quiet
get_verify_drc_mode -limit -quiet
set_verify_drc_mode -disable_rules {} -check_ndr_spacing auto -check_only default -check_same_via_cell false -exclude_pg_net false -ignore_trial_route false -ignore_cell_blockage false -use_min_spacing_on_block_obs auto -report cache_memory.drc.rpt -limit 1000
verify_drc
set_verify_drc_mode -area {0 0 0 0}
verifyConnectivity -type all -error 1000 -warning 50
verifyProcessAntenna -report cache_memory.antenna.rpt -error 1000
saveDesign cache_memory.enc
streamOut cache.gds -libName DesignLib -units 2000 -mode ALL
zoomBox -383.22000 27.61750 1028.43650 710.10200
