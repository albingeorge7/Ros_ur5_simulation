from flask import Flask
from dash import Dash, html, dcc, Input, Output, State
from transformers import AutoModelForCausalLM, AutoTokenizer
import torch

server = Flask(__name__)

app = Dash(__name__, server=server, url_base_pathname='/')


MODEL_NAME = "bigcode/starcoder"
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

model = AutoModelForCausalLM.from_pretrained(MODEL_NAME).to(device)
tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME)

# context for the LLM
motion_api_context = """
The Motion API allows for controlling a UR5 robot. 
Available functions include:
1. move_robot_joint(point1, point2, joint_velocity, joint_acceleration)
2. move_robot_cartesian(pose1, pose2, linear_velocity, linear_acceleration)
Each function controls the robot's motion in joint or Cartesian space.
"""


app.layout = html.Div([
    html.H1("Motion API Co-Pilot", style={'textAlign': 'center'}),
    html.Div([
        dcc.Textarea(
            id="user-prompt",
            placeholder="Describe your motion task...",
            style={'width': '100%', 'height': 150},
        ),
        html.Button("Generate Code", id="generate-button", n_clicks=0, style={'margin-top': '10px'}),
        html.Div(id="loading-placeholder"),
        html.Pre(id="generated-code",
                 style={'backgroundColor': '#f4f4f4', 'padding': '10px', 'whiteSpace': 'pre-wrap'}),
    ], style={'width': '80%', 'margin': '0 auto', 'textAlign': 'center'}),
])


# Callback to handle code generation
@app.callback(
    Output("generated-code", "children"),
    Output("loading-placeholder", "children"),
    Input("generate-button", "n_clicks"),
    State("user-prompt", "value"),
    prevent_initial_call=True
)
def generate_code(n_clicks, user_prompt):
    if not user_prompt:
        return "Please enter a description of the motion task.", ""

    # Generate code using the LLM
    prompt = motion_api_context + "\n" + user_prompt
    inputs = tokenizer.encode(prompt, return_tensors="pt").to(device)

    # Generate code with a maximum token limit
    outputs = model.generate(inputs, max_length=300, temperature=0.7, num_return_sequences=1)
    generated_code = tokenizer.decode(outputs[0], skip_special_tokens=True)

    return generated_code, ""


if __name__ == '__main__':
    app.run_server(debug=True)
