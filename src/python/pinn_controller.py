import torch
import torch.nn as nn
import numpy as np
from flask import Flask, request, jsonify
import json

class QuadrotorPINN(nn.Module):
    def __init__(self):
        super().__init__()
        # Input: [x, y, z, vx, vy, vz, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]
        self.network = nn.Sequential(
            nn.Linear(12, 64),
            nn.Tanh(),
            nn.Linear(64, 32),
            nn.Tanh(),
            nn.Linear(32, 4)  # Output: [thrust, roll_cmd, pitch_cmd, yaw_cmd]
        )
        
    def forward(self, x):
        # Normalize input
        x_norm = self.normalize_input(x)
        # Get network output
        u = self.network(x_norm)
        # Denormalize and enforce constraints
        return self.denormalize_output(u)
    
    def normalize_input(self, x):
        # Simple min-max normalization
        return (x - self.input_mean) / self.input_std
    
    def denormalize_output(self, u):
        # Denormalize and enforce control constraints
        u_denorm = u * self.output_std + self.output_mean
        # Enforce control limits
        u_denorm[0] = torch.clamp(u_denorm[0], 0, 1)  # Thrust between 0 and 1
        u_denorm[1:] = torch.clamp(u_denorm[1:], -1, 1)  # Attitude commands between -1 and 1
        return u_denorm

class PINNController:
    def __init__(self):
        self.model = QuadrotorPINN()
        self.load_model()
        
    def load_model(self):
        try:
            self.model.load_state_dict(torch.load('pinn_model.pth'))
            self.model.eval()
        except:
            print("No pre-trained model found. Using untrained model.")
    
    def predict(self, state):
        # Convert state to tensor
        state_tensor = torch.FloatTensor(state)
        # Get control commands
        with torch.no_grad():
            control = self.model(state_tensor)
        return control.numpy()

# Flask server for MATLAB interface
app = Flask(__name__)
pinn_controller = PINNController()

@app.route('/control', methods=['POST'])
def get_control():
    try:
        state = request.json['state']
        control = pinn_controller.predict(state)
        return jsonify({
            'status': 'success',
            'control': control.tolist()
        })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500

if __name__ == '__main__':
    app.run(host='localhost', port=5000) 