import json
with open(r'C:\Users\Administrator\.cursor\projects\c-Users-Administrator-Desktop-WORK-ardupilot-ubuntu\agent-tools\4da55787-f065-449f-80dc-c95b87354a48.txt', 'r') as f:
    data = json.load(f)
    times = data['times_s']
    for i, t in enumerate(times):
        if 130 < t < 150:
            print(f't={t:.2f} Noise={data["series"]["Noise"][i]} RSSI={data["series"]["RSSI"][i]} RemNoise={data["series"]["RemNoise"][i]} RemRSSI={data["series"]["RemRSSI"][i]}')
