import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
import numpy as np
import concurrent.futures
from threading import Lock
import time
#import random
from TMotorCANControl.servo_can import TMotorManager_servo_can as sc

from concurrent.futures import ThreadPoolExecutor
import threading

class MyNode(Node):

    def __init__(self):
        super().__init__('AkxCnts')
        # Subscriber to the /mytopic
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/motorCnt',
            self.listener_callback,
            10)
        
        # Publisher to publish the modified message
        # self.publisher = self.create_publisher(Float32MultiArray, '/mytopic_modified', 10)
                # New publisher for motor positions
        self.position_publisher = self.create_publisher(Float64MultiArray, '/AKmotor_pose', 10)

        ## 31 0, middle, right: -
        #
        
        # ID list and corresponding values
        self.ID_list = [31,32, 41] #, 31] #, 41, 32]
        #self.ID_list = [23,13]
        self.offsets = {
            13:0,
            23: 0,
            31: 0,
            41: 0,
            32: 0,
        }
        self.factors = {
            13:1,
            23: 1,
            31: 1,
            41: 1,
            32: 2,
        }
        self.velocities = {
            13:1000,
            23:500,
            31: 1000,
            41: 1000,
            32: 1000,
        }
        self.maxAbsVelocity = {
            13:7000,
            23: 7000,
            31: 7000,
            41: 7000,
            32: 7000,
        }
        self.minPos = {
            13:-360,
            23:-360,
            31: -90,
            41: -90,
            32: -90,
        }
        self.maxPos = {
            13:360,
            23:360,
            31: 60,
            41: 90,
            32: 90,
        }
        # self.currents = { # this is not important
        #     23: 1,
        #     31: 0.5,
        #     41: 0.6,
        #     32: 0.4,
        # }
        self.accelerations = {
            13:100,
            23: 100,
            31: 100,
            41: 100,
            32: 100,
        }
        self.motor_types = {
            13:'AK80-8',
            23:'AK80-8',
            31: 'AK80-8',
            41: 'AK80-64',
            32: 'AK80-8',
        }
        self.positions = {
            13:0,
            23:0,
            31: 0,
            41: 0,
            32: 0,
        }

        self.stop = False
        self.motors = {}

        ##self.thread_pool_executor = concurrent.futures.ThreadPoolExecutor(max_workers=len(self.ID_list))
        ##self.active_tasks = {}  # Dictionary to track active tasks per ID
        ##self.task_lock = Lock()  # Lock to synchronize access to the active_tasks dictionary
        # Timer to publish motor positions every second
        self.create_timer(1, self.publish_motor_positions)

        self.initialize_motors()


        # Create a single-thread executor for asynchronous motor looping
        self.thread_pool_executor = ThreadPoolExecutor(max_workers=1)

        # Start looping motors asynchronously
        self.future_loop = self.thread_pool_executor.submit(self.loop_motors_async)


        self.create_timer(1, self.publish_motor_positions)



        

    def initialize_motors(self):
        # Initialize the motors based on the provided IDs and types
        for motor_id in self.ID_list:
            type_motor = self.motor_types[motor_id]
            self.motors[motor_id] = sc(motor_type=type_motor, motor_ID=motor_id)
            self.motors[motor_id].__enter__()
            #time.sleep(0.5)
            self.motors[motor_id].enter_position_velocity_control()
            self.motors[motor_id].update()

            time.sleep(1)

            # self.get_logger().info(f"the motor id is: {motor_id} and the the type is {self.motor_types[motor_id]} ")
            # # updating the position of the motor once
            # #self.motors[motor_id].update()  ## this although essential but caues to move motor to zero value
            
            # # self.motors[motor_id].position =  0 #self.positions[motor_id] ## updating the position for the publisher to safely update 
            # # self.motors[motor_id].acceleration =  0 #self.accelerations[motor_id]
            # # self.motors[motor_id].velocity =  0 #self.velocities[motor_id]
            # # self.motors[motor_id].update()
            # # time.sleep(0.2)
            # self.positions[motor_id] = float(self.motors[motor_id].position)
            # self.motors[motor_id].position =  self.positions[motor_id] ## updating the position for the publisher to safely update 
            # self.motors[motor_id].acceleration = 100 #self.accelerations[motor_id]
            # self.motors[motor_id].velocity =  0 #self.velocities[motor_id]
            # self.motors[motor_id].update()
            # time.sleep(0.2)
            
            self.positions[motor_id] = float(self.motors[motor_id].position)  ## assuming this the current position
            

    def loop_motors_async(self):
        """Asynchronously loop through all motors to continuously update their positions, accelerations, and velocities."""
        while not self.stop:
            for motor_id in self.ID_list:
                try:
                    # Calculate the sign based on the relative positions
                    if self.positions[motor_id] >= self.motors[motor_id].position:
                        signDir = 1  # Moving towards a more positive position
                    else:
                        signDir = -1  # Moving towards a more negative position
                    # Update the motor's state
                    self.motors[motor_id].position = self.positions[motor_id]
                    self.motors[motor_id].acceleration = self.accelerations[motor_id]
                    self.motors[motor_id].velocity = signDir * self.velocities[motor_id]
                    self.motors[motor_id].update()


                    # # Determine the target position and the current position in degrees
                    # current_position_deg = self.motors[motor_id].position
                    # target_position_deg = self.positions[motor_id]

                    # # Calculate the difference between the target and current positions
                    # position_difference = abs(target_position_deg - current_position_deg)

                    # # Set a threshold for position proximity
                    # threshold = 0.2

                    # # Check if the current position is within the threshold of the target position
                    # if position_difference <= threshold:
                    #     # If within 0.2 degrees, stop the motor by setting acceleration and velocity to zero
                    #     self.motors[motor_id].acceleration = 0
                    #     self.motors[motor_id].velocity = 0
                    # else:
                    #     # Otherwise, set the acceleration and velocity based on desired values and direction
                    #     signDir = -1 if current_position_deg > target_position_deg else 1
                    #     self.motors[motor_id].acceleration = self.accelerations[motor_id]
                    #     self.motors[motor_id].velocity = signDir * self.velocities[motor_id]

                    
                    # Log for debugging purposes
                    self.get_logger().info(f"Updated motor {motor_id}: position {self.positions[motor_id]}, acceleration {self.accelerations[motor_id]}, velocity {self.velocities[motor_id]}")

                except Exception as e:
                    self.get_logger().error(f"Error updating motor {motor_id}: {e}")

            time.sleep(0.025)  # Sleep for a short duration to prevent busy waiting


    def __del__(self):
        
        # Ensure the motor is properly closed when the object is deleted
        for motor_id in self.ID_list:
            if self.motors[motor_id] is not None:
                self.self.motors[motor_id].__exit__(None, None, None)  # Manually call __exit__ to clean up
        
                # Stop the loop and shutdown the executor gracefully
        self.stop = True
        if self.thread_pool_executor:
            self.thread_pool_executor.shutdown(wait=True)
        
        

    def listener_callback(self, msg):
        # Log the received message for debugging
        self.get_logger().info(f"Received: {msg.data}")
        
        # Process the Position and Velocity values
        cmdsToMotors= self.construct_matrix_from_data(msg.data, msg.layout)

        
        if cmdsToMotors is not None:
            self.get_logger().info(f"Constructed Matrix:\n{cmdsToMotors}")
            
            # Process the matrix
            #processed_data = self.process_matrix(cmdsToMotors)
            self.process_matrix(cmdsToMotors)
            # # Create a new message to publish
            # new_msg = Float32MultiArray()
            # new_msg.layout = msg.layout
            # new_msg.data = processed_data.flatten().tolist()  # Flatten the matrix back to a list
            
            # # Publish the new message
            # self.publisher.publish(new_msg)
            # self.get_logger().info(f"Published: {new_msg.data}")
    
    def construct_matrix_from_data(self, data, layout):
        # Extract the labels
        labels = [dim.label for dim in layout.dim]
        
        # Define valid combinations
        valid_combinations = [
            ['ID', 'Position'],
            ['ID', 'Position', 'Velocity'],
            ['ID', 'Velocity']
        ]
        
        # Check if the received labels match any valid combination
        if labels not in valid_combinations:
            self.get_logger().error(
                f"Invalid label combination: {labels}. "
                f"Valid combinations are: {valid_combinations}"
            )
            return None
        
        # Get the number of rows from the first dimension's size
        num_rows = layout.dim[0].size / len(labels)
        num_cols = len(labels)
        print(f"here is numbe or rows {num_rows} and here is number of coloumns {num_cols}" )
        #return
        num_rows = layout.dim[0].size / len(labels)

        # Check if the result has a decimal part
        if num_rows.is_integer():
            num_rows = int(num_rows)  # Convert to integer if no decimal part
        else:
            self.get_logger().error(
                f"Invalid size given (check label). "
                f"Invalid number of data")
            return None
            

        # Initialize an empty matrix
        matrix = np.zeros((num_rows, num_cols))
        
        # Populate the matrix based on the number of columns
        for i in range(num_rows):
            for j in range(num_cols):
                index = i * num_cols + j
                matrix[i][j] = data[index]

        print(f"the matrix is {matrix} " )
        # Filter rows based on ID list
        matrix = np.array([row for row in matrix if int(row[0]) in self.ID_list])
        if matrix.size == 0:
            self.get_logger().warn("No valid ID found in the data.")
            return None

        # Apply transformations based on labels
        if 'Position' in labels:
            for row in matrix:
                id_val = int(row[0])
                row[1] = (row[1] + self.offsets[id_val]) * self.factors[id_val]
                # Check with minPos and maxPos
                row[1] = max(self.minPos[id_val], min(row[1], self.maxPos[id_val]))
        
        if 'Velocity' in labels:
            for row in matrix:
                id_val = int(row[0])
                # Check the absolute value of velocity
                if abs(row[-1]) > self.maxAbsVelocity[id_val]:
                    row[-1] = np.sign(row[-1]) * self.maxAbsVelocity[id_val]
        
        # If only ID and Position are present, add velocities as the third column
        if labels == ['ID', 'Position']:
            vel_direction_col = np.array([self.velocities[int(row[0])] for row in matrix])
            matrix = np.column_stack((matrix, vel_direction_col))
            self.get_logger().info("Added velocities as the third column.")


        return matrix

    def process_matrix(self, matrix):
        for row in matrix:
            self.send_command_to_motor(row)

            # motor_id = int(row[0])
            # with self.task_lock:
            #     if motor_id in self.active_tasks:
            #         self.get_logger().info(f"Motor ID {motor_id} is already processing. Skipping new command.")
            #         continue

            #     # Submit a new task and store the future in the dictionary
            #     future = self.thread_pool_executor.submit(self.send_command_to_motor, row)
            #     self.active_tasks[motor_id] = future

            #     # Attach a callback to clean up the dictionary when the task is done
            #     future.add_done_callback(lambda fut, motor_id=motor_id: self.cleanup_task(motor_id))



    def send_command_to_motor(self, row):

        try:
            motor_id = int(row[0])
            goal_position = row[1]
            velocity = row[2] if len(row) > 2 else None
            
            if len(row) > 2:


                self.positions[motor_id] = goal_position ## updating the position for the publisher to safely update 
            
                self.velocities[motor_id] = velocity

                self.get_logger().info(f"Sending command to motor {motor_id}: goal_position {goal_position}, Velocity {velocity}")
                # cnt = 0 # for later safety mechanism
                # motor = self.motors[motor_id]
                
                # motor.enter_position_velocity_control()
                # for t in range(1, 100):
                #     motor.velocity = velocity
                #      #
                #     prevPosition = float(motor.position)
                #     motor.position = goal_position
                #     motor.acceleration = self.accelerations[motor_id]
                #     motor.update()
                #     time.sleep(0.5)
                #     self.positions[motor_id] = float(motor.position)
                #     self.get_logger().info(f'\nMotor {motor_id} current position: {motor.position} degrees.')
                #     if abs(prevPosition - goal_position) <= 0.1:
                #         break
                #     if abs(prevPosition - self.positions[motor_id]) <= 0.1:
                #         cnt += 1
                #         if cnt == 1:
                #             self.get_logger().info(f"motorID: {motor_id} got stuck, cnt: {cnt}")
                #             break
                        
                           
                    
            else:

                self.get_logger().info(f"Sending command to motor {motor_id}:, Velocity {velocity}")
                # cnt = 0 # for later safety mechanism
                # motor = self.motors[motor_id]

                # for t in range(1, 10):
                #     motor.velocity = velocity
                #     #self.positions[motor_id] = float(motor.position) ## better not to update it here
                #     #motor.position = position
                #     #motor.acceleration = self.accelerations[motor_id]
                #     motor.update()
                #     time.sleep(0.1)
                #     # self.get_logger().info(f'\nMotor {motor_id} current position: {motor.position} degrees.')
                #     # if abs(float(motor.position) - position) <= 0.1:
                #     #     break

                # # motor.enter_velocity_control()
                # # motor.velocity = velocity
                # # motor.update()
                self.get_logger().info("Velocity control has not been written yet")
            
            # Return some result or status
            return "Success"
        except Exception as e:
         self.get_logger().error(f"Error controlling motor {motor_id}: {str(e)}")
         return "Failure"

    def cleanup_task(self, motor_id):
        with self.task_lock:
            if motor_id in self.active_tasks:
                del self.active_tasks[motor_id]
                self.get_logger().info(f"Motor ID {motor_id} processing completed. Task removed from active list.")

    def publish_motor_positions(self):
        # Simulate gathering positions for each motor
        positions = []
        cnt = 0

        for motor_id in self.ID_list:
            # #self.get_logger().info(f"Published motor positions: {motor_id}")
            # # Simulate a delay for each motor, ranging from 0 to 2 seconds
            # #self.motors[motor_id].update()  ## this although essential but caues to move motor to zero value
            # #self.positions[motor_id]
            # #position = float(self.motors[motor_id].position)

            # # temp = position - self.positions[motor_id]
            # # if temp > 0.1:
            # #     self.get_logger().info(f"motorID: {motor_id} is moving by {temp}")

            # for i in range(2):
            #     prevPosition = self.motors[motor_id].position 
            #     self.motors[motor_id].position = self.positions[motor_id] ## updating the position for the publisher to safely update 
            #     self.motors[motor_id].acceleration = self.accelerations[motor_id]
            #     self.motors[motor_id].velocity = self.velocities[motor_id]
            #     self.motors[motor_id].update()
            #     time.sleep(0.2)
            currentPosition = self.motors[motor_id].position
            #position = self.motors[motor_id].position
            iqaxis =  self.motors[motor_id].current_qaxis
            velocity = self.motors[motor_id].velocity
            acc = self.motors[motor_id].acceleration
            positions.extend([float(motor_id)*10000000, currentPosition, iqaxis, velocity, acc])


            # if abs(prevPosition - currentPosition) <= 0.05:
            #     cnt += 1
            #     if cnt == 1:
            #         self.get_logger().info(f"motorID: {motor_id} got stuck OR achieved the position, cnt: {cnt}")
            #         cnt = 0 
            #         self.positions[motor_id] = currentPosition
    
        # Publish the gathered positions
        msg = Float64MultiArray()
        msg.data = positions
        self.position_publisher.publish(msg)



        #self.get_logger().info(f"Published motor positions: {msg.data}")

        # for motor_id in self.ID_list:
        #     #self.get_logger().info(f"Published motor positions: {motor_id}")
        #     # Simulate a delay for each motor, ranging from 0 to 2 seconds
        #     self.positions[motor_id] = float(self.motors[motor_id].position)
        #     #time.sleep(random.uniform(0, 0.1))
        #     time.sleep(0.05)
        #     # Generate a random position
        #     position = self.positions[motor_id]
        #     positions.extend([float(motor_id), position])
        
        # # Publish the gathered positions
        # msg = Float64MultiArray()
        # msg.data = positions
        # self.position_publisher.publish(msg)
        # self.get_logger().info(f"Published motor positions: {msg.data}")



def main(args=None):


    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
