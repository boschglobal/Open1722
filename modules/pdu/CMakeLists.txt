# PDU parsing library
add_library(open1722pdu SHARED
    "src/avtp/pdu/CommonHeader.c"
    "src/avtp/pdu/Crf.c"
    "src/avtp/pdu/Rvf.c"
    "src/avtp/pdu/Udp.c"
    "src/avtp/pdu/Utils.c"
    "src/avtp/pdu/aaf/CommonStream.c"
    "src/avtp/pdu/aaf/PcmStream.c"
    "src/avtp/pdu/acf/Can.c"
    "src/avtp/pdu/acf/CanBrief.c"
    "src/avtp/pdu/acf/Common.c"
    "src/avtp/pdu/acf/Ntscf.c"
    "src/avtp/pdu/acf/Sensor.c"
    "src/avtp/pdu/acf/SensorBrief.c"
    "src/avtp/pdu/acf/Tscf.c"
    "src/avtp/pdu/cvf/Cvf.c"
    "src/avtp/pdu/cvf/H264.c"
    "src/avtp/pdu/cvf/Jpeg2000.c"
    "src/avtp/pdu/cvf/Mjpeg.c")
target_include_directories(open1722pdu PRIVATE "include")
