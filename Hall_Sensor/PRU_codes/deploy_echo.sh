#! /bin/bash
# Copyright (C) 2016 Pierrick Rauby <PierrickRauby - pierrick.rauby@gmail.com>
#
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the
#	  distribution
#
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#############################################################################

HEADER=P8_
PIN_IN=45
PIN_OUT=46

echo "**** Starting ****"
echo "building Halt_PRU"

alias cd_PRU0_Halt="cd PRU0_Halt"
cd_PRU0_Halt

make clean
make
echo "copying files into /lib/firmware/am335x-pru0-fw"
cp gen/*.out /lib/firmware/am335x-pru0-fw
echo "done"
echo "building PRU_Hall"

alias cd_PRU1_Hall="cd ../PRU1_Hall"
cd_PRU1_Hall

echo "-Configuring pinmux"
	config-pin -a $HEADER$PIN_IN pruin
	config-pin -q $HEADER$PIN_IN
	config-pin -a $HEADER$PIN_OUT pruout
	config-pin -q $HEADER$PIN_OUT

make clean
make
echo "copying files into /lib/firmware/am335x-pru1-fw"
cp gen/*.out /lib/firmware/am335x-pru1-fw
echo "done"

echo "Rebooting PRUs"
echo 'stop' > /sys/class/remoteproc/remoteproc1/state
echo 'stop' > /sys/class/remoteproc/remoteproc2/state
echo 'start' > /sys/class/remoteproc/remoteproc1/state
echo 'start' > /sys/class/remoteproc/remoteproc2/state

echo "done"
