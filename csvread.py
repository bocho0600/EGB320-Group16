import csv

# Reading the CSV file and storing the rows in a list
with open("Order_1.csv", mode="r", encoding='utf-8-sig') as csv_file:
    item_info = list(csv.reader(csv_file))
    for row in item_info:
      print(row)
    # Initialize robot_instruction with the first row as headers and placeholders for subsequent rows
    robot_instruction = [['Aisle', 'Bay', 'Side', 'Height', 'Name']]
    
    for row in item_info[1:]:  # Skip the header row from item_info
        if row[1] in ["0", "1"]:
            aisle = '1'
            if row[1] == "0":
                side = 'Left'
            else: side = 'Right'
        elif row[1] in ["2", "3"]:
            aisle = '2'
            if row[1] == "2":
                side = 'Left'
            else: side = 'Right'
        elif row[1] in ["4", "5"]:
            if row[1] == "4":
                side = 'Left'
            else: side = 'Right'
            aisle = '3'
        else:
            continue  # Skip rows that do not match any of the conditions\
        # Append the transformed row to robot_instruction
        robot_instruction.append([aisle, row[1], side, row[3], row[4]])
        

# Printing the first row (header) and all instructions
print("Robot Instructions:")
for instruction in robot_instruction:
    print(instruction)
