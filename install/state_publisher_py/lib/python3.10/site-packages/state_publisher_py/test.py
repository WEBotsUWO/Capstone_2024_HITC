import csv

results = []
with open('/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/joint_angle_trajectory.csv', newline='') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=',')
        for row in csvreader:
            results.append(row)
print(len(results))

print(results[998][3])