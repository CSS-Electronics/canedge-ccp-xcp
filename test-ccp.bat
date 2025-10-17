python canedge_daq.py tests/output/ecu1-ccp.dbc tests/output/ecu1-ccp.json tests/measurement-files/sample-ccp.csv --a2l tests/a2l-files/ECU-sample-file-ccp.a2l
python update_existing_config.py tests/output/ecu1-ccp.json tests/config-schema-files/config-01.09.json true --input_schema_file tests/config-schema-files/schema-01.09.json
