#!/bin/bash
set -ex
make -j"$(nproc)"

flash() {
  openocd -f interface/stlink.cfg -f target/stm32g0x.cfg \
    -c "hla_serial $2" \
    -c "init;reset halt;flash write_image erase build/$1.hex; reset run;shutdown"
}

if [ "$1" = "receiver" ] || [ -z "$1" ]; then
  flash receiver 066EFF313541483043183849
fi

if [ "$1" = "sensor" ] || [ -z "$1" ]; then
  flash sensor 066EFF363946433043085449
fi

echo "OK"
