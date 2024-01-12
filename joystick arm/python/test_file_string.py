import csv
class arm:
    def __init__(self):
        self.theta1=30
        self.theta2=90
        self.theta3=60
        self.angles = []
    def read_from_csv(self, filename='output.csv'):
        # Read from CSV file
        with open(filename, 'r') as csvfile:
            reader = csv.reader(csvfile)
            # Assuming each row is a list of values
            for row in reader:
                print(row)
if __name__ == "__main__":
    arm_instance = arm()
    arm_instance.read_from_csv()