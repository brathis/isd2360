# Arduino ISD2360
This project contains an Arduino software library to interact with Nuvoton ChipCorder® ISD2360 devices via SPI.
It further includes an uploader consisting of a Python script and an Arduino sketch which may be used to upload
ISD2360 firmware images to the device.

## Setup
### Using the library
The library makes use of the ISD2360's `RDY/BSYB` pin for flow control. Applications must use this pin in addition
to the required SPI pins. The pin number of the `RDY/BSYB` pin must be passed to the constructor as well as the
pin number of the chip-select pin.

## Using the uploader
The uploader has so far only been tested on Linux using Python 3.8. The following non-standard dependencies are required:
  - [`tqdm`](https://github.com/tqdm/tqdm)
  - [`pyserial`](https://github.com/pyserial/pyserial)
When using pip, you may use the provided `requirements.txt` file to automatically install the dependencies as follows:
```
pip install -r ./uploader/requirements.txt
```

## References
  - [ISD2360 Datasheet](https://www.nuvoton.com/export/resource-files/EN_ISD2360_Datasheet_Rev1-0.pdf)
  - [ISD2360 Design Guide](https://www.nuvoton.com/export/resource-files/EN_ISD2360_Datasheet_Rev1-0.pdf)
