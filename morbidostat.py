# Starting work to get rid of the arduino altogether. Running a Dash server on a raspberry pi and saving all data to the sd card (json) and, in the future, a database.

import dash
import dash_html_components as html
import dash_core_components as dcc
import dash_auth
import plotly.graph_objs as go
import json

from dash.dependencies import Output, Input

VALID_USERNAME_PASSWORD_PAIRS = {
    'pi': '###'
}

ALLOWED_TYPES = (
    "text", "number", "password", "email", "search",
    "tel", "url", "range", "hidden",
)

from adafruit_motorkit import MotorKit # Install library with 'pip3 install adafruit-circuitpython-motorkit' and don't forget to enable I2C on the PI
from adafruit_motor import stepper # Import stepper library to step the motors

# Initialize motors
try:
    hat = MotorKit(address=96)
    print('added steppers to hat 1')
    motor = {} # Will not be generated if an error above?
    motor[0] = hat.stepper1
    motor[1] = hat.stepper2
    try:
        for x in range(97, 128):
            try:
                print('attempt to add hat ' + str(x))
                y = MotorKit(address=x)
                motor[len(motor)+1] = y.stepper1
                motor[len(motor)+1] = y.stepper2
            except:
                print('no hat on '+str(x))
    except:
        print('Only one hat found')
except:
    print('Error: No I2C devices found, please check all connections')

# import mysql.connector as mariadb # To use a database or not to

# mariadb_connection = mariadb.connect(user='python_user', password='some_pass', database='core_settings')
# core_settings = mariadb_connection.cursor()
# try:
#   core_settings.execute("SELECT ALL)
# except mariadb.Error as error:
#   print("Error: {}".format(error))

# mariadb_connection = mariadb.connect(user='python_user', password='some_pass', database='data')
# core_data = mariadb_connection.cursor()
# try:
#   core_data.execute("SELECT ALL)
# except mariadb.Error as error:
#   print("Error: {}".format(error))

external_stylesheets = ["https://codepen.io/chriddyp/pen/bWLwgP.css"]

app = dash.Dash(__name__, external_stylesheets=external_stylesheets, suppress_callback_exceptions=True)
auth = dash_auth.BasicAuth(
    app,
    VALID_USERNAME_PASSWORD_PAIRS
)
app.css.append_css({'external_url': '/assets/style.css'})
app.server.static_folder = 'assets'

colors = {
    'background': '#FFFFFF',
    'text': '#7FDBFF'
}


def build_banner():
    return html.Div(
        id="banner",
        className="banner",
        style={
        },
        children=[
            html.Div(
                id="banner-text",
                children=[
                    html.H5("Morbidostat")
                ]
            ),
        ]
    )


def build_home_tab():
    return dcc.Tab(label='Home', children=[
                html.Div([
                dcc.Graph(id='home_graph',
                          figure={'data': [{
                              'x': eval('data["time"]'),
                              'y': eval('data["data1"]'),
                              'name': 'Vial 1'},
                              {'x': eval('data["time"]'),
                               'y': eval('data["data2"]'),
                               'name': 'Vial 2'}],
                              'layout': go.Layout(
                                  xaxis={'type': 'log', 'title': 'Time (sec)'},
                                  yaxis={'title': 'Vial OD'},
                                  margin={'l': 40, 'b': 40, 't': 10, 'r': 10},
                                  legend={'x': 0, 'y': 1},
                                  hovermode='closest'
                              )
                          })]),

                html.H5(children=["-"], style={'width': '100%', 'backgroundColor': '#77b0b1', 'color': '#77b0b1'}),

                html.Div([
                    html.Div([
                        html.Div([
                            dcc.Checklist(id='enablepump1',
                                          options=[
                                              {'label': 'Pump 1', 'value': 1},
                                          ], value=[]),
                        ], style={}),
                        html.Div(id='test'),
                        html.Div([
                            dcc.Slider(id='slider1', disabled=True,
                                       min=0, max=100, step=0.5,),]),
                        html.Div(id='output1'),
                    ]),
                    html.Div([
                        html.Div([
                            dcc.Checklist(id='enablepump2',
                                          options=[
                                              {'label': 'Pump 2', 'value': 'Pump2'},
                                          ], value=[]),
                        ], style={}),
                        html.Div([
                            dcc.Slider(id='slider2', disabled=True,
                                       min=0, max=100, step=0.5,)]),
                        html.Div(id='output2'),
                    ]),
                    html.Div([
                        html.Div([
                            dcc.Checklist(id='enablepump3',
                                          options=[
                                              {'label': 'Pump 3', 'value': 1},
                                          ], value=[]),
                        ], style={}),
                        html.Div([
                            dcc.Slider(id='slider3', disabled=True,
                                       min=0, max=100, step=0.5,),]),
                        html.Div(id='output3'),
                    ]),
                    html.Div([
                        html.Div([
                            dcc.Checklist(id='enablepump4',
                                          options=[
                                              {'label': 'Pump 4', 'value': 1},
                                          ], value=[]),
                        ], style={}),
                        html.Div([
                            dcc.Slider(id='slider4', disabled=True,
                                       min=0, max=100, step=0.5,)]),
                        html.Div(id='output4'),
                    ]),
                ], style={'columnCount': 2})])


def build_settings_tab():
    return dcc.Tab(label='Settings', children=[
                html.Button('Motors Off', id='bMotors_off', style={'color': 'red'}, n_clicks=0),
                html.Div([
                    html.Div([
                        html.Div([
                            'Record on vial(s):',
                            [
                            html.Button('Vial {}'.format(_), id='bVial{}_rec', style={'margin': '2%'})
                            for _ in range(1,len(motor))]
                            ],
                            style={'width': '50%'})
                    ]),
                    html.Div([
                        html.Div([
                            'Set filename: ',
                            dcc.Input(
                                id="input_filename",
                                placeholder="input",)
                        ])
                    ])
                ], style={'columnCount': 2})
            ])


markdown_text = '#'

data = {
    'time': [1, 2, 3],
    'data1': [6, 5, 10],
    'data2': [4, 3, 1],
}

app.layout = html.Div(
    id="big-app-container",
    style={'backgroundColor': colors['background']},
    children=[
        markdown_text,
        build_banner(),

        html.H5(children=["-"], style={'width': '100%', 'backgroundColor': '#77b0b1', 'color': '#77b0b1'}),
        dcc.Tabs(id="tabs", children=[
            build_home_tab(),
            build_settings_tab(),
        ])
    ])


@app.callback(
    [Output('bVial{}_rec'.format(_), 'style') for _ in range(1,len(motor))],
    [Input('bVial{}_rec'.format(_), 'n_clicks') for _ in range(1,len(motor))])
def update_rec(value):
    if value %2 == 0:
        return dict(color = 'black', backgroundColor = 'white')
    else:
        return dict(color = 'black', backgroundColor = 'green')


@app.callback(
    [Output('enablepump1', 'value'),
     Output('enablepump2', 'value'),
     Output('enablepump3', 'value'),
     Output('enablepump4', 'value'),
     Output('bMotors_off', 'style'),
     Output('bMotors_off', 'children')],
    [Input('bMotors_off', 'n_clicks')])
def motors_off(val):
    if val % 2 == 0:
        style = {'color': 'red'}
        children = 'Motors Off'
    else:
        style = {'color': 'green'}
        children = 'Motor(s) On'
    # with open('json/pumps.json', 'r') as file:
    #     settings = json.load(file)
    #     for p in settings:
    #         settings[eval(p["enabled"])] = "0"
    #     settings.seek(0)
    #     json.dump(settings, 'json/pumps.json')
    return 0, 0, 0, 0, style, children


@app.callback(
    dash.dependencies.Output('output1', 'children'),
    [dash.dependencies.Input('slider1', 'value')])
def update_output(value):
    return 'Pump is set to {}%'.format(value)


@app.callback(
    dash.dependencies.Output('output2', 'children'),
    [dash.dependencies.Input('slider2', 'value')])
def update_output(value):
    return 'Pump is set to {}%'.format(value)


@app.callback(
    dash.dependencies.Output('output3', 'children'),
    [dash.dependencies.Input('slider3', 'value')])
def update_output(value):
    return 'Pump is set to {}%'.format(value)


@app.callback(
    dash.dependencies.Output('output4', 'children'),
    [dash.dependencies.Input('slider4', 'value')])
def update_output(value):
    return 'Pump is set to {}%'.format(value)


@app.callback(
    dash.dependencies.Output('slider1', 'disabled'),
    [dash.dependencies.Input('enablepump1', 'value')])
def en_pump1(value):
    if value:
        return False
    else:
        return True


@app.callback(
    dash.dependencies.Output('slider2', 'disabled'),
    [dash.dependencies.Input('enablepump2', 'value')])
def en_pump2(value):
    if value:
        return False
    else:
        return True


@app.callback(
    dash.dependencies.Output('slider3', 'disabled'),
    [dash.dependencies.Input('enablepump3', 'value')])
def en_pump3(value):
    if value:
        return False
    else:
        return True


@app.callback(
    dash.dependencies.Output('slider4', 'disabled'),
    [dash.dependencies.Input('enablepump4', 'value')])
def en_pump4(value):
    if value:
        return False
    else:
        return True


if __name__ == '__main__':
    app.run_server(dev_tools_hot_reload=True, host='0.0.0.0')


# def build_motor_sliders(num):
# # This won't work unless I adjust callbacks too.
# # Maybe an autogenerated python script calling the dash script?
#     for x in range(num):
#         html.Div([
#             html.Div([
#                 dcc.Checklist(id='enablepump1',
#                           options=[
#                               {'label': 'Pump 1', 'value': 1},
#                           ], value=[]),
#                 ], style = {}),
#                 html.Div(id='test'),
#                 html.Div([
#                 dcc.Slider(id='slider1', disabled=True,
#                            min=0, max=100, step=0.5, ),]),
#                 html.Div(id='output1')])
#     return

