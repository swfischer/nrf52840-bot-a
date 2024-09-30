# nrf52840-bot-a
Saving code of a robot project built based on a Nordic nRF52840 microcontroller

## Getting started

### Initialization
Create a workspace folder. Run the following command:
```shell
mkdir nrf52840-bot-a
cd nrf52840-bot-a
```
Create a python virtual environment and install "west":
```shell
python3 -m venv .venv
source .venv/bin/activate
pip install west
```
Initialize the example code and download all the necessary components:
```shell
west init -m https://github.com/swfischer/nrf52840-bot-a.git .
west update
west zephyr-export
```
Install the remaining python tools:
```shell
pip install -r zephyr/scripts/requirements.txt
```

### Building
The most common means of building the project is:
```shell
cd apps
west build main -b adafruit_feather_x -p
```
