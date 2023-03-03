# read experiments_4.log line by line
#format the data into a dataframe
import numpy as np
import pandas as pd

raw_data = pd. DataFrame(columns = ['scene', 'target_object', 'planner_type', 'run', 'success', 'stability'])

with open('experiments_8.log') as f:
    content = f.readlines()
    for line in content:
        if "Scene: " in line:
            scene = line.split(": ")[1].strip()
        if "Target object: " in line:
            target_object = line.split(": ")[1].strip()
        if "Planner type: " in line:
            planner_type = line.split(": ")[1].strip()
        if "Run" in line:
            run = int(line[4])
            success = line.split(":")[-1].strip()
            if success == "True":
                success = True
            else:
                success = False
        if "Difference in rotation" in line:
            rotation_text = line.split(": ")[1].strip()
            if rotation_text == "None":
                stability = 0
            elif success == False:
                stability = 0
            else:
                rotation = float(rotation_text)
                # rotation = np.min([rotation, 2*np.pi-rotation])
                stability = 1/rotation
            
            # store the data in a dataframe
            raw_data = raw_data.append({'scene': scene, 'target_object': target_object, 'planner_type': planner_type, 'run': run, 'success': success, 'stability': stability}, ignore_index=True)
print(raw_data)

# for each object and each planner type, calculate the average success rate the average inverse rotation.
# if the trial is not successful, the inverse rotation is set to 0

data = pd. DataFrame(columns = ['object', 'planner_type', 'success_rate', 'average_stability'])
for object in raw_data['target_object'].unique():
    for planner_type in raw_data['planner_type'].unique():
        df = raw_data[(raw_data['target_object'] == object) & (raw_data['planner_type'] == planner_type)]
        success_rate = df['success'].mean()
        average_stability = df['stability'].mean()
        if df["scene"].unique().size >= 1:
            scene = df['scene'].unique()[0]
        data = data.append({'scene': scene, 'object': object, 'planner_type': planner_type, 'success_rate': success_rate, 'average_stability': average_stability}, ignore_index=True)

# print the data and write it to a latex table
print(data)
data.to_latex('experiments_6.tex', index=False)
            
            
