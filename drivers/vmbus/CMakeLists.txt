add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/netvscres.dll" DESTINATION reactos/system32 FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/vmbuspipe.dll" DESTINATION reactos/system32 FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/vmbusres.dll" DESTINATION reactos/system32 FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/vmbusvideod.dll" DESTINATION reactos/system32 FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/vmstorfltres.dll" DESTINATION reactos/system32 FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/drivers/hyperkbd.sys" DESTINATION reactos/system32/drivers FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/drivers/netvsc50.sys" DESTINATION reactos/system32/drivers FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/drivers/storvsc.sys" DESTINATION reactos/system32/drivers FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/drivers/vmbkmcl.sys" DESTINATION reactos/system32/drivers FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/drivers/vmbus.sys" DESTINATION reactos/system32/drivers FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/drivers/vmbushid.sys" DESTINATION reactos/system32/drivers FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/drivers/vmbusvideom.sys" DESTINATION reactos/system32/drivers FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/drivers/vmgencounter.sys" DESTINATION reactos/system32/drivers FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/drivers/vms3cap.sys" DESTINATION reactos/system32/drivers FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/drivers/vmstorfl.sys" DESTINATION reactos/system32/drivers FOR all)
add_cd_file(FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARCH}/drivers/winhv.sys" DESTINATION reactos/system32/drivers FOR all)
add_driver_inf(
    vmbus
    ${ARCH}/gencounter.inf
    ${ARCH}/hyperkbd.inf
    ${ARCH}/netvsc.inf
    ${ARCH}/s3cap.inf
    ${ARCH}/storflt.inf
    ${ARCH}/storvsc.inf
    ${ARCH}/vmbus.inf
    ${ARCH}/vmbushid.inf
    ${ARCH}/vmbusvideo.inf
    ${ARCH}/vmic.inf
    ${ARCH}/vpcinull.inf)
