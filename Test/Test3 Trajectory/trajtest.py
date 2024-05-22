import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import glob
import pandas as pd

# Function to calculate distance between a point and a line
def distance_point_to_line(p, p1, p2):
    return np.linalg.norm(np.cross(p2 - p1, p1 - p)) / np.linalg.norm(p2 - p1)

unit_graph = "mm"



#!test 1
row1 = np.array([[0,1620],[1620,2750],[2750,3385],[3385,4580],[4580,5453],[5453,5937], [5937,6500],[6500, 7585],[7585,8254],[8254,8842]])
#!test 2
row2 = np.array([[2760,3724],[3724,4586],[4586,5010],[5010,5645],[5645,6433],[6433,7085],[7085,7603],[7603,8220],[8220,8541],[8541, 10583]])
#!test 3
row3 = np.array([[659,1656],[1656,2592],[2592,3000],[3000,3630],[3630,4573],[4573,5251],[5251,5836],[5836,6646],[6646,7078],[7078, 7461]])
#!test 4
row4 = np.array([[6542,6906],[6906,7980],[7980,8949],[8949,9802],[9802,10662],[10662,11510],[11510,12137],[12137,13019],[13019,13638],[13638, 14439]])
#!test 5
row5 = np.array([[219,1385],[1385,2483],[2483,3020],[3020,3817],[3817,4820],[4820,5800],[5800,6542],[6542,7302],[7302,7758],[7758, 8524]])
#!test 6
row6 = np.array([[188,575],[575,1499],[1499,2073],[2073,2722],[2722,3342],[3342,4095],[4095,4732],[4732,5441],[5441,6091],[6091, 6737]])
#!test 7
row7 = np.array([[452, 1808],[1808,3044],[3044,3518],[3518,4275],[4275,5086],[5086,5942],[5942,6717],[6717,7483],[7483,8013],[8013,8709]])

rows = [row1,row2,row3,row4,row5,row6,row7]

for j in range(len(rows)):
    row = rows[j]
    
    # Reference trajectory data
    x_ref_1 = np.array([[1016, 1016], [1016, 1420], [1420, 358], [358, -1082], [-1082, -1780], [-1780, -1362],[-1362,-87],[-87,-110],[-110,-528],[-528,1211]])
    y_ref_1 = np.array([[-970, -970], [-970 ,370], [370, 1439], [1439, 1158], [1158, 488], [488, -997], [-997, -1494], [-1494, 33], [33, 1031], [1031, -92]])
    z_ref_1 = np.array([[0,500],[500,1000],[1000,1200],[1200,900],[900,600],[600,1100],[1100,1500],[1500,1800],[1800,1300],[1300,900]])


    x_drone = np.zeros(len(x_ref_1), dtype=object) # Initialize empty arrays
    y_drone = np.zeros(len(x_ref_1), dtype=object)
    z_drone = np.zeros(len(x_ref_1), dtype=object)

    for i in range(len(x_ref_1)):
        x_drone[i], y_drone[i], z_drone[i] = np.loadtxt(f"Trajectory test data\\Trajectory_data_{j+1}.csv",
                                                        delimiter=';', unpack=True, skiprows=row[i][0], max_rows=row[i][1]-row[i][0])


    # Create a new figure for the plots
    fig = plt.figure(figsize=(10, 10))

    # 3D plot of the trajectories
    ax1 = fig.add_subplot(211, projection='3d')
    ax1.set_xlabel(f'X axis [{unit_graph}]')
    ax1.set_ylabel(f'Y axis [{unit_graph}]')
    ax1.set_zlabel(f'Z axis [{unit_graph}]')
    ax1.legend()

    distancessss = []
    colorssss = []

    listofstuff = []
    listofallstuff = np.array([["mean","std","truefalse"]])
    emptylist = ["","",""]    

    for i in range(len(x_ref_1)):
        # Number of points to interpolate
        num_points = len(x_drone[i])
        x1 = x_ref_1[i]
        y1 = y_ref_1[i]
        z1 = z_ref_1[i]
        
        x2 = x_drone[i]
        y2 = y_drone[i]
        z2 = z_drone[i]

        # Interpolating values for the first trajectory
        x1_interp = np.linspace(x1[0], x1[1], num_points)
        y1_interp = np.linspace(y1[0], y1[1], num_points)
        z1_interp = np.linspace(z1[0], z1[1], num_points)

        # Calculate distances
        distances = np.array([distance_point_to_line(np.array([x2[j], y2[j], z2[j]]), np.array([x1[0], y1[0], z1[0]]), np.array([x1[1], y1[1], z1[1]])) for j in range(num_points)])
        colors = ['red' if distance > 250 else 'darkgreen' for distance in distances]
        
        # Plot trajectories
        ax1.plot(x1_interp, y1_interp, z1_interp, label=f'Trajectory {i+1} Ref', linestyle='dashed')
        ax1.plot(x2, y2, z2, label=f'Trajectory {i+1} Drone')
        ax1.scatter(x2, y2, z2, c=colors, s=2)  # Scatter plot with colors based on distance

        
        mean = np.mean(distances)
        std = np.std(distances)
        #check for true false
        truefalse = [True if distance < 250 else False for distance in distances]
        if False in truefalse:
            truefalse = False
        else:
            truefalse = True
        listofstuff.append([mean,std,truefalse])
        
        distancessss.extend(distances)
        colorssss.extend(colors)

    print(*listofstuff, sep='\n')
    
    listofstuff = pd.DataFrame(listofstuff, columns=["mean","std","truefalse"])
    listofstuff.to_csv(f"Trajectory test data\\Trajectory_data_results.csv", sep=";", index=False, header=False, mode='a')
      
    # Comparison of X coordinates
    ax2 = fig.add_subplot(212)
    ax2.scatter(np.arange(len(distancessss)) / 100, distancessss, color=colorssss, zorder=1, s=5)
    
    # Error margin
    error_margin = 250
    ax2.fill_between(np.arange(len(distancessss)) / 100, 0, error_margin, color='green', alpha=0.3, label='Error Margin', zorder=0)

    ax2.scatter([], [], color='darkgreen', label='Within Error Margin', zorder=2)
    ax2.scatter([], [], color='red', label='Out of Error Margin', zorder=2)

    ax2.set_title(f'Distance to Line [{unit_graph}]')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel(f'Distance to Line [{unit_graph}]')
    max_time = len(distancessss) / 100
    ax2.set_xticks(np.arange(0, max_time + 1, 5))
    ax2.legend()

    # Adjust layout
    plt.tight_layout()

    # Show the plot
    plt.show()
    

