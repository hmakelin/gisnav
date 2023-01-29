Jetson Nano & PX4/Pixhawk
____________________________________________________

The following example describes how to run GISNav on a Jetson Nano in a PX4 HIL simulation on a Pixhawk board.

Prerequisites
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
* Connect your Jetson Nano to your desktop (simulation host) computer via Ethernet cable (see :ref:`Onboard computer`
  for more information).
* Connect your Pixhawk board to your desktop (simulation host) computer via USB cable.
* Install a bootloader on your Pixhawk board if your board does not yet have one. See your board manufacturer's
  instructions on how to load one onto your specific board.
* (1) Install the `https://github.com/hmakelin/PX4-Autopilot.git`_ custom fork of PX4-Autopilot which includes
  required modifications to the microDDS bridge ``dds_topics.yaml`` configuration file, or (2) configure the bridge
  yourself (see :ref:`PX4-ROS 2 bridge topic configuration`).

  .. _https://github.com/hmakelin/PX4-Autopilot.git: https://github.com/hmakelin/PX4-Autopilot.git

Upload PX4 firmware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
See the `PX4 uploading firmware instructions`_ for how to upload your development version of PX4 onto your Pixhawk
board. To find the ``make`` target for your specific board, list all options with the ``make list_config_targets``
command:

.. _PX4 uploading firmware instructions: https://docs.px4.io/main/en/dev_setup/building_px4.html#uploading-firmware-flashing-the-board

.. code-block:: bash

    cd ~/PX4-Autopilot
    make list_config_targets

The output will look like the following:

.. code-block:: text
    :caption: Example output of ``make list_config_targets`` command

    hmakelin@hmakelin-Nitro-AN515-54:~/PX4-Autopilot$ make list_config_targets
    airmind_mindpx-v2[_default]
    ark_can-flow_canbootloader
    ark_can-flow[_default]
    ark_can-gps_canbootloader
    ark_can-gps[_default]
    ark_cannode_canbootloader
    ark_cannode[_default]
    ark_can-rtk-gps_canbootloader
    ark_can-rtk-gps_debug
    ark_can-rtk-gps[_default]
    ark_fmu-v6x_bootloader
    ark_fmu-v6x[_default]
    atl_mantis-edu[_default]
    av_x-v1[_default]
    beaglebone_blue[_default]
    bitcraze_crazyflie21[_default]
    bitcraze_crazyflie[_default]
    cuav_can-gps-v1_canbootloader
    cuav_can-gps-v1[_default]
    cuav_nora_bootloader
    cuav_nora[_default]
    cuav_x7pro_bootloader
    cuav_x7pro[_default]
    cuav_x7pro_test
    cubepilot_cubeorange_bootloader
    cubepilot_cubeorange[_default]
    cubepilot_cubeorange_test
    cubepilot_cubeyellow[_default]
    cubepilot_io-v2[_default]
    diatone_mamba-f405-mk2[_default]
    emlid_navio2[_default]
    flywoo_gn-f405[_default]
    freefly_can-rtk-gps_canbootloader
    freefly_can-rtk-gps[_default]
    holybro_can-gps-v1_canbootloader
    holybro_can-gps-v1[_default]
    holybro_durandal-v1_bootloader
    holybro_durandal-v1[_default]
    holybro_kakutef7[_default]
    holybro_kakuteh7_bootloader
    holybro_kakuteh7[_default]
    holybro_pix32v5[_default]
    matek_gnss-m9n-f4_canbootloader
    matek_gnss-m9n-f4[_default]
    matek_h743_bootloader
    matek_h743[_default]
    matek_h743-mini_bootloader
    matek_h743-mini[_default]
    matek_h743-slim_bootloader
    matek_h743-slim[_default]
    modalai_fc-v1[_default]
    modalai_fc-v2_bootloader
    modalai_fc-v2[_default]
    modalai_voxl2[_default]
    modalai_voxl2-io[_default]
    modalai_voxl2-slpi[_default]
    mro_ctrl-zero-classic_bootloader
    mro_ctrl-zero-classic[_default]
    mro_ctrl-zero-f7[_default]
    mro_ctrl-zero-f7-oem[_default]
    mro_ctrl-zero-h7_bootloader
    mro_ctrl-zero-h7[_default]
    mro_ctrl-zero-h7-oem_bootloader
    mro_ctrl-zero-h7-oem[_default]
    mro_pixracerpro_bootloader
    mro_pixracerpro[_default]
    mro_x21-777[_default]
    mro_x21[_default]
    nxp_fmuk66-e[_default]
    nxp_fmuk66-e_socketcan
    nxp_fmuk66-v3[_default]
    nxp_fmuk66-v3_socketcan
    nxp_fmuk66-v3_test
    nxp_fmurt1062-v1[_default]
    nxp_ucans32k146_canbootloader
    nxp_ucans32k146_cyphal
    nxp_ucans32k146[_default]
    omnibus_f4sd[_default]
    omnibus_f4sd_icm20608g
    px4_fmu-v2[_default]
    px4_fmu-v2_fixedwing
    px4_fmu-v2_lto
    px4_fmu-v2_multicopter
    px4_fmu-v2_rover
    px4_fmu-v3[_default]
    px4_fmu-v3_test
    px4_fmu-v4[_default]
    px4_fmu-v4pro[_default]
    px4_fmu-v4pro_test
    px4_fmu-v4_test
    px4_fmu-v5_cryptotest
    px4_fmu-v5_cyphal
    px4_fmu-v5_debug
    px4_fmu-v5[_default]
    px4_fmu-v5_lto
    px4_fmu-v5_protected
    px4_fmu-v5_stackcheck
    px4_fmu-v5_test
    px4_fmu-v5_uavcanv0periph
    px4_fmu-v5x[_default]
    px4_fmu-v5x_test
    px4_fmu-v6c_bootloader
    px4_fmu-v6c[_default]
    px4_fmu-v6u_bootloader
    px4_fmu-v6u[_default]
    px4_fmu-v6x_bootloader
    px4_fmu-v6x[_default]
    px4_io-v2[_default]
    px4_raspberrypi[_default]
    px4_sitl[_default]
    px4_sitl_nolockstep
    px4_sitl_replay
    px4_sitl_test
    raspberrypi_pico[_default]
    scumaker_pilotpi_arm64
    scumaker_pilotpi[_default]
    sky-drones_smartap-airlink[_default]
    spracing_h7extreme[_default]
    uvify_core[_default]

Then choose your appropriate board for the following examples. The example below assumes you have a
`NXP FMUK66-E board`_.

.. _NXP FMUK66-E board: https://docs.px4.io/main/en/flight_controller/nxp_rddrone_fmuk66.html

.. code-block:: bash
    :caption: Upload PX4 to NXP FMU66K board

    git submodule update --recursive
    make distclean
    make nxp_fmuk66-e_default upload

Run HIL simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Follow the steps in `PX4 HIL simulation instructions`_.

.. _PX4 HIL simulation instructions: https://docs.px4.io/main/en/simulation/hitl.html

Once you have the HIL simulation running, login to your Jetson Nano and start the onboard services (as described in
:ref:`Onboard computer`) just like in the SITL simulation case:

.. code-block:: bash
    :caption: Run GISNav and GIS server on onboard computer

    cd ~/colcon_ws
    make build-px4
    make up-px4