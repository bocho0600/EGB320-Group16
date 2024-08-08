import csv

# Reading the CSV file and storing the rows in a list
class CSVReader:
    def __init__(self, file_name):
        self.file_name = file_name
        self.item_info = []
        self.robot_instruction = [['Aisle', 'Bay', 'Side', 'Height', 'Name']]

    def read_csv(self):
        with open(self.file_name, mode="r", encoding='utf-8-sig') as csv_file:
            self.item_info = list(csv.reader(csv_file))
            for row in self.item_info:
                print(row)
            return self.item_info
    
    def RobotInstruction(self):
        for row in self.item_info[1:]:
            for row in self.item_info[1:]:  # Skip the header row from item_info
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
                    continue  # Skip rows that do not match any of the conditions
                # Append the transformed row to robot_instruction
                self.robot_instruction.append([aisle, row[1], side, row[3], row[4]])
                print("Robot Instructions:")
                for instruction in self.robot_instruction:
                    print(instruction)
        return self.robot_instruction
    


csv = CSVReader('items.csv')
csv.read_csv()
instruction = csv.RobotInstruction()

if __name__ == "__main__":
    csv = CSVReader('items.csv')
    csv.read_csv()
    csv.RobotInstruction()