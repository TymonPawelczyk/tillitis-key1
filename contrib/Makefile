build:
	docker build -t key1 .

run:
	docker run --mount type=bind,source="`pwd`/../",target=/tillitis-key1 -w /tillitis-key1 -it key1

USB_DEVICE?=/dev/ttyACM0

# For tillitis-key1-apps development
app:
	docker run \
	--mount type=bind,source="`pwd`/../../tillitis-key1-apps",target=/tillitis-key1-apps -w /tillitis-key1-apps \
	--device=$(USB_DEVICE) \
	-it key1

test-loop:
	docker run \
	--mount type=bind,source="`pwd`/../../tillitis-key1-apps",target=/tillitis-key1-apps -w /tillitis-key1-apps \
	--device=$(USB_DEVICE) \
	-it key1 \
	/tillitis-key1-apps/test-loop.sh
