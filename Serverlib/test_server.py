#!/usr/bin/env python3
"""
AGV Connection Test Script
Chạy trên máy AGV (100.93.107.29) để test kết nối tới server
"""

import socket
import time
import json
import threading

def test_basic_connection(host="100.93.107.119", port=8080, timeout=5):
    """Test basic TCP connection"""
    print(f"Testing basic connection to {host}:{port}...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        
        start_time = time.time()
        result = sock.connect_ex((host, port))
        connect_time = time.time() - start_time
        
        if result == 0:
            print(f"✓ Connection successful in {connect_time:.3f}s")
            sock.close()
            return True
        else:
            print(f"✗ Connection failed: Error code {result}")
            return False
            
    except Exception as e:
        print(f"✗ Connection exception: {e}")
        return False

def test_agv_protocol_communication(host="100.93.107.119", port=8080):
    """Test full AGV protocol communication"""
    print(f"\nTesting AGV protocol communication...")
    
    try:
        # Tạo kết nối
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)
        sock.connect((host, port))
        print("✓ TCP connection established")
        
        # Tạo test message theo protocol AGV
        test_data = {
            "current_x": 1.5,
            "current_y": 2.3,
            "current_angle": 45.0,
            "plc_registers": {
                "D100": 1,
                "D101": 0,
                "D102": 0,
                "D103": 1500
            },
            "lidar_points": [
                {"x": 1.0, "y": 0.5},
                {"x": 1.2, "y": 0.3},
                {"x": 0.8, "y": 0.7}
            ],
            "status": {
                "is_moving": True,
                "is_safe": True,
                "battery_level": 85.5,
                "current_speed": 1500,
                "timestamp": int(time.time() * 1000)
            }
        }
        
        # Tạo message theo format AGV
        message_content = {
            "type": 1,  # STATUS_UPDATE
            "payload": json.dumps(test_data),
            "timestamp": int(time.time() * 1000)
        }
        
        json_str = json.dumps(message_content)
        full_message = f"$$START$${len(json_str)}|{json_str}$$END$$"
        
        # Gửi message
        print(f"Sending test message ({len(full_message)} bytes)...")
        sock.send(full_message.encode('utf-8'))
        print("✓ Test message sent")
        
        # Đợi response (nếu có)
        print("Waiting for response...")
        sock.settimeout(5)
        try:
            response = sock.recv(1024)
            if response:
                print(f"✓ Received response: {len(response)} bytes")
                print(f"Response preview: {response[:100]}...")
            else:
                print("No response received (this might be normal)")
        except socket.timeout:
            print("No response within 5 seconds (this might be normal)")
        
        sock.close()
        print("✓ Protocol test completed successfully")
        return True
        
    except Exception as e:
        print(f"✗ Protocol test failed: {e}")
        return False

def test_continuous_connection(host="100.93.107.119", port=8080, duration=30):
    """Test continuous connection stability"""
    print(f"\nTesting continuous connection for {duration} seconds...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        print("✓ Initial connection established")
        
        start_time = time.time()
        message_count = 0
        
        while time.time() - start_time < duration:
            try:
                # Gửi heartbeat message
                heartbeat = {
                    "type": 5,  # HEARTBEAT
                    "payload": json.dumps({
                        "timestamp": int(time.time() * 1000),
                        "message_count": message_count
                    }),
                    "timestamp": int(time.time() * 1000)
                }
                
                json_str = json.dumps(heartbeat)
                full_message = f"$$START$${len(json_str)}|{json_str}$$END$$"
                
                sock.send(full_message.encode('utf-8'))
                message_count += 1
                
                # Đợi 2 giây
                time.sleep(2)
                
                # Kiểm tra connection vẫn còn
                sock.settimeout(0.1)
                try:
                    data = sock.recv(1024, socket.MSG_DONTWAIT)
                    if data:
                        print(f"Received data: {len(data)} bytes")
                except (socket.timeout, BlockingIOError):
                    pass  # No data available, that's ok
                
            except Exception as e:
                print(f"✗ Error during continuous test: {e}")
                break
        
        elapsed = time.time() - start_time
        print(f"✓ Continuous connection test completed")
        print(f"  Duration: {elapsed:.1f}s")
        print(f"  Messages sent: {message_count}")
        print(f"  Average rate: {message_count/elapsed:.1f} msg/s")
        
        sock.close()
        return True
        
    except Exception as e:
        print(f"✗ Continuous connection test failed: {e}")
        return False

def check_network_environment():
    """Check network environment"""
    print("\n=== Network Environment Check ===")
    
    import subprocess
    import platform
    
    try:
        # Kiểm tra IP local
        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
        if result.returncode == 0:
            local_ips = result.stdout.strip().split()
            print(f"Local IP addresses: {local_ips}")
            
            # Kiểm tra có IP 100.93.107.29 không
            if "100.93.107.29" in local_ips:
                print("✓ AGV IP address (100.93.107.29) found")
            else:
                print(f"⚠ Expected AGV IP (100.93.107.29) not found in {local_ips}")
    except:
        pass
    
    # Ping server
    print(f"\nPinging server 100.93.107.119...")
    try:
        result = subprocess.run(['ping', '-c', '4', '100.93.107.119'], 
                               capture_output=True, text=True)
        if result.returncode == 0:
            print("✓ Server is reachable")
            # Extract ping statistics
            lines = result.stdout.split('\n')
            for line in lines:
                if 'packet loss' in line or 'time=' in line:
                    print(f"  {line.strip()}")
        else:
            print("✗ Server is not reachable")
            print(result.stderr)
    except Exception as e:
        print(f"✗ Ping test failed: {e}")
    
    # Check routing
    print(f"\nChecking route to server...")
    try:
        result = subprocess.run(['ip', 'route', 'get', '100.93.107.119'], 
                               capture_output=True, text=True)
        if result.returncode == 0:
            print(f"Route: {result.stdout.strip()}")
        else:
            print("Route information not available")
    except:
        pass

def main():
    """Main test function"""
    print("AGV Connection Test Script")
    print("=" * 50)
    print("Testing connection from AGV (100.93.107.29) to Server (100.93.107.119:8080)")
    
    # 1. Check network environment
    check_network_environment()
    
    # 2. Test basic connection
    print(f"\n=== Basic Connection Test ===")
    basic_ok = test_basic_connection()
    
    if not basic_ok:
        print("\n❌ Basic connection failed. Check:")
        print("1. Server is running on 100.93.107.119:8080")
        print("2. Network connectivity between machines")
        print("3. Firewall settings")
        return
    
    # 3. Test AGV protocol
    print(f"\n=== AGV Protocol Test ===")
    protocol_ok = test_agv_protocol_communication()
    
    if not protocol_ok:
        print("\n❌ Protocol test failed. Check:")
        print("1. Server is using correct AGV message protocol")
        print("2. JSON parsing on server side")
        return
    
    # 4. Test continuous connection
    print(f"\n=== Continuous Connection Test ===")
    continuous_ok = test_continuous_connection(duration=10)
    
    if continuous_ok:
        print(f"\n✅ All tests passed! AGV can connect to server successfully.")
        print(f"\nNext steps:")
        print(f"1. Run the full C++ AGV program")
        print(f"2. Monitor connection logs")
        print(f"3. Test actual data transmission")
    else:
        print(f"\n⚠ Continuous connection has issues. Check network stability.")

if __name__ == "__main__":
    main()