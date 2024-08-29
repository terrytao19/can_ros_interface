import can

def main():
    # Create a bus instance
    bus = can.interface.Bus(channel='can2', bustype='socketcan')

    print("Listening on can0... Press Ctrl+C to exit.")
    
    try:
        while True:
            # Read a message from the CAN bus
            message = bus.recv()
            
            # Print the raw message
            print(f"ID: {message.arbitration_id:03X}, DLC: {message.dlc}, Data: {message.data.hex()}")

    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting...")
    finally:
        # Close the bus
        bus.shutdown()

if __name__ == "__main__":
    main()
