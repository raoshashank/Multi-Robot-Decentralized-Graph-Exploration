import yaml

connection_config = open("../config/connection.yaml", "r")

yaml_config = connection_config.read()
connection_config.close()

data = yaml.load( yaml_config )

print data
