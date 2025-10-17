python canedge_daq.py tests/output/ecu1-xcp.dbc tests/output/ecu1-xcp.json tests/measurement-files/sample-xcp.csv --a2l tests/a2l-files/ECU-sample-file-xcp.a2l
python update_existing_config.py tests/output/ecu1-xcp.json tests/config-schema-files/config-01.09.json true --input_schema_file tests/config-schema-files/schema-01.09.json
