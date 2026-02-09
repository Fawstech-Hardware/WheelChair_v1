"""
Voice Control Test Script
Tests speech recognition without requiring Vision/Actuator hardware.
Run: python voice_test.py
"""
import speech_recognition as sr
import pyttsx3
import threading
import queue
import time
from difflib import SequenceMatcher
from enum import Enum

# --- Mock LocationID for testing ---
class LocationID(Enum):
    HAZARD = 0
    BATHROOM = 5 
    HALLWAY = 6
    BEDROOM = 3 
    LIVING_ROOM = 7
    KITCHEN = 9

# --- Simple Logger ---
class Logger:
    COLORS = {
        'INFO': '\033[94m',     # Blue
        'SUCCESS': '\033[92m',  # Green
        'WARNING': '\033[93m',  # Yellow
        'ERROR': '\033[91m',    # Red
        'DEBUG': '\033[90m',    # Gray
        'RESET': '\033[0m'
    }
    
    @staticmethod
    def log(level, message):
        timestamp = time.strftime("%H:%M:%S")
        color = Logger.COLORS.get(level, Logger.COLORS['RESET'])
        print(f"{color}[{timestamp}] [{level:8}] {message}{Logger.COLORS['RESET']}")

# --- Voice Controller ---
class VoiceTestController:
    """Standalone voice controller for testing without hardware"""
    
    COMMANDS = {
        # Navigation commands
        "kitchen": ("navigate", LocationID.KITCHEN),
        "go to kitchen": ("navigate", LocationID.KITCHEN),
        "take me to kitchen": ("navigate", LocationID.KITCHEN),
        "bedroom": ("navigate", LocationID.BEDROOM),
        "go to bedroom": ("navigate", LocationID.BEDROOM),
        "take me to bedroom": ("navigate", LocationID.BEDROOM),
        "bathroom": ("navigate", LocationID.BATHROOM),
        "go to bathroom": ("navigate", LocationID.BATHROOM),
        "hallway": ("navigate", LocationID.HALLWAY),
        "go to hallway": ("navigate", LocationID.HALLWAY),
        "living room": ("navigate", LocationID.LIVING_ROOM),
        "go to living room": ("navigate", LocationID.LIVING_ROOM),
        
        # Control commands
        "stop": ("control", "stop"),
        "halt": ("control", "stop"),
        "emergency": ("control", "stop"),
        
        # Mode switching
        "manual": ("mode", "manual"),
        "manual mode": ("mode", "manual"),
        "auto": ("mode", "auto"),
        "automatic": ("mode", "auto"),
        
        # Manual movement
        "forward": ("move", "forward"),
        "go forward": ("move", "forward"),
        "backward": ("move", "backward"),
        "back": ("move", "backward"),
        "left": ("move", "left"),
        "turn left": ("move", "left"),
        "right": ("move", "right"),
        "turn right": ("move", "right"),
    }
    
    FUZZY_THRESHOLD = 0.70
    
    def __init__(self):
        self.engine = pyttsx3.init()
        self.recognizer = sr.Recognizer()
        self.speech_queue = queue.Queue()
        self.running = True
        self.manual_mode = False
        
        # List available microphones and find Realtek
        print("\n📋 Available Microphones:")
        mic_index = None
        for i, mic_name in enumerate(sr.Microphone.list_microphone_names()):
            marker = ""
            # Find FIRST Realtek microphone (not already selected)
            if mic_index is None and "realtek" in mic_name.lower() and "microphone" in mic_name.lower():
                mic_index = i
                marker = " ✅ SELECTED"
            print(f"   [{i}] {mic_name}{marker}")
        print()
        
        # Use Realtek microphone if found, otherwise default
        if mic_index is not None:
            self.microphone = sr.Microphone(device_index=mic_index)
            print(f"🎤 Using: Microphone index {mic_index} (Realtek)\n")
        else:
            self.microphone = sr.Microphone()
            print("🎤 Using: Default microphone\n")
        
        # Configure recognizer - LOWER threshold for better sensitivity
        self.recognizer.energy_threshold = 100  # Lowered from 300
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.5  # Faster response
        
        # Start TTS worker
        threading.Thread(target=self._speech_worker, daemon=True).start()
    
    def _speech_worker(self):
        while self.running:
            try:
                text = self.speech_queue.get(timeout=1)
                self.engine.say(text)
                self.engine.runAndWait()
            except queue.Empty:
                pass
            except Exception as e:
                Logger.log("ERROR", f"TTS Error: {e}")
    
    def speak(self, text):
        Logger.log("INFO", f"🗣️ Speaking: {text}")
        self.speech_queue.put(text)
    
    def _fuzzy_match(self, heard_text):
        best_match = None
        best_score = 0
        
        for command in self.COMMANDS.keys():
            if command in heard_text:
                return command, 1.0
            
            score = SequenceMatcher(None, heard_text, command).ratio()
            if score > best_score:
                best_score = score
                best_match = command
        
        if best_score >= self.FUZZY_THRESHOLD:
            return best_match, best_score
        return None, 0
    
    def _execute_command(self, action_type, action_data):
        if action_type == "navigate":
            location_name = action_data.name.replace("_", " ").title()
            self.speak(f"Navigating to {location_name}")
            Logger.log("SUCCESS", f"🎯 Would navigate to: {location_name}")
            
        elif action_type == "control":
            if action_data == "stop":
                self.speak("Stopping")
                Logger.log("WARNING", "🛑 Emergency Stop triggered")
                
        elif action_type == "mode":
            if action_data == "manual":
                self.manual_mode = True
                self.speak("Manual mode activated")
                Logger.log("INFO", "🎮 Manual Mode ON")
            else:
                self.manual_mode = False
                self.speak("Autonomous mode activated")
                Logger.log("INFO", "🤖 Auto Mode ON")
                
        elif action_type == "move":
            if not self.manual_mode:
                self.speak("Please enable manual mode first")
                return
            self.speak(f"Moving {action_data}")
            Logger.log("DEBUG", f"🕹️ Would move: {action_data}")
    
    def run(self):
        print("\n" + "="*60)
        print("🎤 VOICE CONTROL TEST")
        print("="*60)
        print("Speak commands into your microphone.")
        print("Say 'quit' or 'exit' to stop.")
        print("="*60 + "\n")
        
        # Calibrate microphone
        try:
            with self.microphone as source:
                Logger.log("INFO", "🎤 Calibrating microphone (2 seconds)...")
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
                Logger.log("SUCCESS", "🎤 Microphone ready!")
        except Exception as e:
            Logger.log("ERROR", f"Microphone failed: {e}")
            return
        
        self.speak("Voice control ready. Say a command.")
        
        while self.running:
            try:
                with self.microphone as source:
                    Logger.log("DEBUG", "🎤 Listening...")
                    audio = self.recognizer.listen(source, timeout=10, phrase_time_limit=5)
                
                heard_text = self.recognizer.recognize_google(audio).lower().strip()
                Logger.log("INFO", f"👂 Heard: \"{heard_text}\"")
                
                # Check for exit commands
                if any(word in heard_text for word in ["quit", "exit", "bye", "goodbye"]):
                    self.speak("Goodbye!")
                    Logger.log("INFO", "👋 Exiting...")
                    self.running = False
                    break
                
                # Match command
                matched_cmd, score = self._fuzzy_match(heard_text)
                
                if matched_cmd:
                    action_type, action_data = self.COMMANDS[matched_cmd]
                    Logger.log("SUCCESS", f"✅ Matched: \"{matched_cmd}\" (score: {score:.0%})")
                    self._execute_command(action_type, action_data)
                else:
                    Logger.log("WARNING", f"❓ Unknown command: \"{heard_text}\"")
                    self.speak("Command not recognized")
                
            except sr.WaitTimeoutError:
                Logger.log("DEBUG", "⏱️ No speech detected")
            except sr.UnknownValueError:
                Logger.log("DEBUG", "🔇 Could not understand speech")
            except sr.RequestError as e:
                Logger.log("ERROR", f"🌐 API Error: {e}")
                time.sleep(2)
            except KeyboardInterrupt:
                Logger.log("INFO", "👋 Interrupted by user")
                break
            except Exception as e:
                Logger.log("ERROR", f"Error: {e}")
                time.sleep(1)
        
        print("\n" + "="*60)
        print("Test complete!")
        print("="*60)


if __name__ == "__main__":
    VoiceTestController().run()
