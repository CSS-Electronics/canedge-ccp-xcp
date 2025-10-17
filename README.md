# CANedge-DAQ: A2L to DAQ 'transmit list' + DBC

## Overview

This project helps you use the CANedge for CCP / XCP on CAN data acquisition:

- Load A2L file(s) 
- Load user-defined text file (e.g. `CSV`) with ECU signal names and event channel numbers 
- Generate DAQ frames required for dynamic DAQ initialization (incl. FD support and optimized packaging)
- Generate CANedge transmit list for setting up dynamic DAQ
- Generate DBC file that enables decoding of the initialized DAQ list
- Load the JSON transmit list into an existing CANedge Configuration File
- Easily combine transmit lists across multiple ECUs

--------

## How to use (single ECU)

- Install Python 3.11+ 
- Install requirements incl. [a2lparser](https://github.com/mrom1/a2lparser) via below:

```
pip install -i https://test.pypi.org/simple/ a2lparser --extra-index-url https://pypi.org/simple/
pip install jsonschema
```

- Prepare your A2L files for a single ECU (do not try to mix A2L files across ECUs)
- Add required A2L measurements in a text file (e.g. `CSV`) structured as below (headers will be ignored):

```
<signalname1>;<event channel index e.g. 1>
<signalname2>;<event channel index e.g. 1>
<signalname3>;<event channel index e.g. 2>
...
```

- Open your [command prompt](https://www.youtube.com/watch?v=bgSSJQolR0E&t=47s) and run below: 

```
python canedge_daq.py path/to/ecu1.dbc path/to/ecu1.json path/to/ecu1.csv --a2l path/to/ecu1_part1.a2l
```

The script will output the generated CANedge transmit list and DBC file. You can easily load the transmit list into an existing Configuration File locally (e.g. from an SD) with optional Rule Schema validation:

```
python update_existing_config.py path/to/ecu1.json path/to/config-01.09.json true --input_schema_file path/to/schema-01.09.json
```

Alternatively, you can load it via the [CANedge Config Editor](https://canlogger.csselectronics.com/canedge-getting-started/ce3/configure-device/) using the 'partial config loader' tool - or via the [canedge-manager](https://github.com/CSS-Electronics/canedge_manager) for OTA updates.

### Test with sample A2L and CSV

You can test the functionality via our sample CCP/XCP files via the `.bat` files or below:

#### CCP sample test 
```
python canedge_daq.py tests/output/ecu1-ccp.dbc tests/output/ecu1-ccp.json tests/measurement-files/sample-ccp.csv --a2l tests/a2l-files/ECU-sample-file-ccp.a2l
python update_existing_config.py tests/output/ecu1-ccp.json tests/config-schema-files/config-01.09.json true --input_schema_file tests/config-schema-files/schema-01.09.json
```

#### XCP sample test 
```
python canedge_daq.py tests/output/ecu1-xcp.dbc tests/output/ecu1-xcp.json tests/measurement-files/sample-xcp.csv --a2l tests/a2l-files/ECU-sample-file-xcp.a2l
python update_existing_config.py tests/output/ecu1-xcp.json tests/config-schema-files/config-01.09.json true --input_schema_file tests/config-schema-files/schema-01.09.json
```

--------

## How to use (multiple ECUs)

The same script functionality can easily be extended for use with multiple ECUs that leverage separate CAN IDs:

- Run the above process once for each ECU (with the relevant A2L and signal files)
- Use the `combine_multiple_ecus.py` script to combine the transmit list JSON files into a single list

See below an example set of commands:

```
python canedge_daq.py path/to/ecu1.dbc path/to/ecu1.json path/to/ecu1.csv --a2l path/to/ecu1_part1.a2l path/to/ecu1_part2.a2l
python canedge_daq.py path/to/ecu2.dbc path/to/ecu1.json path/to/ecu2.csv --a2l path/to/ecu2.a2l
python combine_multiple_ecus.py path/to/ecu1_and_ecu2.json --input_transmit_files path/to/ecu1.json path/to/ecu2.json --offset_delta -500
```

Notes:
- The script warns you if the CAN ID is the same across the JSON files (separate ECUs must use separate IDs)
- The script warns you if the joint transmit list exceeds the max number of frames or data bytes supported
- Optionally use `--offset_delta` to control the time spacing between distinct ECU DAQ initialization sequences

--------

## How to decode the data 

You can decode the DAQ-DTO data using the output DBC file(s) in below tools:

- [asammdf GUI](https://www.csselectronics.com/pages/asammdf-gui-api-mdf4-mf4) (v8.2.5d5+)
- [MF4 decoders](https://www.csselectronics.com/pages/mdf4-decoders-dbc-mf4-parquet-csv) (e.g. for deploying [Grafana dashboards](https://www.csselectronics.com/pages/telematics-dashboard-open-source))

--------

## Other comments

- The script assumes you are using CANedge FW `01.09.01+` (support for longer transmit lists)
- It can be useful to add multiple commands in a `*.bat` file for repeated use
- This script is provided as-is and we do not take responsibility for any issues arising from its use
- If the A2L lacks a CCP/XCP section, the script can instead use a default set of parameters via the `--default_params` input (see the provided CCP/XCP examples)


