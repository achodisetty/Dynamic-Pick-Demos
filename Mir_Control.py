import requests, json

ip = '172.16.10.121'
host = 'http://' + ip + '/api/v2.0.0/'

headers = {}
headers['Content-Type'] = 'application/json'
headers['Authorization'] = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='

# A method to returns the field of choice from the json
# The input must be of json format, the field value (string) to be found
# If the string is found then the corresponding guid is returned else None
def retrive_field(list, field_string, key = 'name', retriveField = 'guid'):
    for item in list:
        if item[key] == field_string:
            return item[retriveField]
    return None


def go_to_location(location_name):

    get_missions = requests.get(host + 'missions', headers=headers)
    missions_list = get_missions.json()

    starting_point = retrive_field(missions_list, 'mobileArm_starting_point')
    table_side = retrive_field(missions_list, 'mobileArm_table_side')
    drop_site = retrive_field(missions_list, 'mobileArm_drop_location')

    if location_name == 'starting_point':
        mission_id = {"mission_id" : starting_point}
    elif location_name == 'table_side':
        mission_id = {"mission_id" : table_side}
    elif location_name == 'drop_site':
        mission_id = {"mission_id" : drop_site}

    requests.post(host + 'mission_queue', json = mission_id, headers = headers)
    wait_flag = True 
    
    while wait_flag:

        mission_queue = requests.get(host + 'mission_queue', headers = headers)
        mission_queue = mission_queue.json()
        mission_status = mission_queue[-1]

        if mission_status['state'] == 'Done':
            wait_flag = False
        
    print(f'{location_name} reached!!!')


# go_to_location('table_side')