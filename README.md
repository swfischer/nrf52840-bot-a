# nrf52840-bot-a

Saving code of a robot project built based on a Nordic nRF52840 microcontroller

## Getting started

### Initialization
The first step is to create a workspace folder. Run the following command:
```shell
mkdir nrf52840-bot-a
cd nrf52840-bot-a
```
Now create a python virtual environment and install "west":
```shell
python3 -m venv .venv
source .venv/bin/activate
pip install west
```
Finally, initialize the example code and download all the necessary components:
```shell
west init -m https://github.com/swfischer/nrf52840-bot-a.git .
west update
```

