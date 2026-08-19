import json
with open(r'C:\Users\Administrator\.cursor\projects\c-Users-Administrator-Desktop-WORK-ardupilot-ubuntu\agent-tools\4da55787-f065-449f-80dc-c95b87354a48.txt', 'r') as f:
    data = json.load(f)
    times = data['times_s']
    for i, t in enumerate(times):
        if 138 < t < 147:
            print(f't={t:.2f}', end=' ')
            for field in data['fields']:
                print(f'{field}={data["series"][field][i]}', end=' ')
            print()
