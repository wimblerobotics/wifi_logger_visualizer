# AI generated: 
# Below is an example of Python code using the matplotlib and seaborn libraries to generate a heatmap
#
# sudo apt install python3-seaborn
#

import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import math
import sqlite3

db_path = '/home/sergei/wifi_ws/src/wifi_strength_logger/database/wifi_data.db'

scale_factor = 1 # map scale, larger value causes finer grid

try:
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    cursor.execute("SELECT x,y,bit_rate,link_quality,signal_level FROM wifi_data")
    #cursor.execute("SELECT * FROM wifi_data")
    #cursor.execute("SELECT count(*) FROM wifi_data")
    rows = np.array(cursor.fetchall())
    rows = np.round(rows, decimals=1) * scale_factor

    conn.close()
    row_count = len(rows)
    #row_count = rows

    print(f"Number of rows: {row_count}")
    print(f"Row 0: {rows[0]}")
    print(f"Row 0 [0] = x: {rows[0][0]}")
    print(f"Row 0 [1] = y: {rows[0][1]}")
    print(f"Row 0 [2] = bit_rate: {rows[0][2]}")
    print(f"Row 0 [3] = link_quality: {rows[0][3]}")
    print(f"Row 0 [4] = signal_level: {rows[0][4]}")

    min_arr = np.min(rows, axis=0)
    min_x = math.floor(min_arr[0])
    min_y = math.floor(min_arr[1])

    max_arr = np.max(rows, axis=0)
    max_x = math.ceil(max_arr[0])
    max_y = math.ceil(max_arr[1])

    print(f"min: {min}")
    print(f"max: {max}")
    
    print(f"min_x: {min_x}  max_x: {max_x}")
    print(f"min_y: {min_y}  max_y: {max_y}")

    # dimensions of the drawing space:
    dim_x = int(math.ceil(max_x)) - int(math.floor(min_x)) + 3
    dim_y = int(math.ceil(max_y)) - int(math.floor(min_y)) + 3
    print(f"drawing space: dim_x: {dim_x}  dim_y: {dim_y}")

    heat_data = np.zeros((dim_y, dim_x)) # somehow x and y are inverted when drawing heatmap

    hd_min = +1000.0
    hd_max = -1000.0
    
    for y in range(dim_y):
        for x in range(dim_x):
            heat_data[y][x] = None # comment this out to see zeroes instead of white space
            for row in rows:
                # find data_x and data_y in rows matching our position (x,y) in drawing space:
                data_x = int(round(row[0]-min_x))+1
                data_y = int(round(row[1]-min_y))+1
                if data_x==x and data_y==y:
                    hd_val = math.floor(row[4] / scale_factor) # signal_level=4; scale it back, round down
                    hd_min = min(hd_min, hd_val)
                    hd_max = max(hd_max, hd_val)
                    heat_data[y][x] = hd_val
                    print(f"{data_x} {data_y}   {x} {y} {heat_data[y][x]}")
        print(f"---- end Y {y}")

except sqlite3.Error as e:
    print(f"Error: retrieving wifi data: {e}")


# Create the heatmap
ax = sns.heatmap(heat_data, annot=True, cmap='coolwarm', fmt=".0f", linewidths=.2, vmin=hd_min, vmax=hd_max)
# https://seaborn.pydata.org/tutorial/color_palettes.html
ax.invert_yaxis()

# Customize the plot (optional)
plt.title('WiFi Signal Strength Heatmap')
plt.xlabel('X-axis Travel')
plt.ylabel('Y-axis Travel')

# Display the heatmap
plt.show()

