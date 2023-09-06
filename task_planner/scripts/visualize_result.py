#!/usr/bin/env python
import json
import rospkg
import matplotlib.pyplot as plt

if __name__ == "__main__":
    '''
    load the json file and visualize the result
    '''

    rospack = rospkg.RosPack()
    # Get the path of the desired package
    package_path = rospack.get_path('task_planner')

    result_json_path = package_path + '/evaluated_data_dir/evaluated_data_pick_and_place_constraint_4sep_mtg.json'

    with open(result_json_path) as f:
        data = json.load(f)

    # find all task planner names
    task_planner_names = list(set([d['task_planner'] for d in data]))

    print "task planner names: ", task_planner_names

    total_time = [0.0 for i in range(len(task_planner_names))]
    total_distance = [0.0 for i in range(len(task_planner_names))]
    experiment_count = [0.0 for i in range(len(task_planner_names))]
    success_count = [0.0 for i in range(len(task_planner_names))]
    times = [[] for i in range(len(task_planner_names))]
    distances = [[] for i in range(len(task_planner_names))]

    for i in range(len(data)):
        for j in range(len(task_planner_names)):
            if data[i]['task_planner'] == task_planner_names[j]:
                experiment_count[j] += 1
                if data[i]['found_solution']:
                    success_count[j] += 1
                    total_time[j] += data[i]['time']
                    total_distance[j] += data[i]['total_distance']
                    times[j].append(data[i]['time'])
                    distances[j].append(data[i]['total_distance'])

    average_time = [0.0 for i in range(len(task_planner_names))]
    average_distance = [0.0 for i in range(len(task_planner_names))]
    success_rate = [0.0 for i in range(len(task_planner_names))]

    for i in range(len(task_planner_names)):
        if success_count[i] == 0:
            print "task planner: ", task_planner_names[i]
            print "always fail"
        else:
            average_time[i] = total_time[i] / success_count[i]
            average_distance[i] = total_distance[i] / success_count[i]
            success_rate[i] = success_count[i] / experiment_count[i]
            print "task planner: ", task_planner_names[i]
            print "average time: ", average_time[i]
            print "average distance: ", average_distance[i]
            print "success rate: ", success_rate[i]

    # visulize the average time, average distance, and success rate for each planner in plt
    plt.figure(0)
    plt.subplot(311)
    plt.boxplot(times)
    plt.ylabel('Time')
    plt.xticks(range(1, len(task_planner_names) + 1), task_planner_names)


    plt.subplot(312)
    plt.boxplot(distances)
    plt.ylabel('Distance')
    plt.xticks(range(1, len(task_planner_names) + 1), task_planner_names)


    plt.subplot(313)
    plt.bar(task_planner_names, success_rate)
    plt.ylabel('Success Rate')
    plt.show()