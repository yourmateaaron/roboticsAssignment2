
function varargout = Dobot_guide(varargin)
% DOBOT_GUIDE MATLAB code for Dobot_guide.fig
%      DOBOT_GUIDE, by itself, creates a new DOBOT_GUIDE or raises the existing
%      singleton*.
%
%      H = DOBOT_GUIDE returns the handle to a new DOBOT_GUIDE or the handle to
%      the existing singleton*.
%
%      DOBOT_GUIDE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DOBOT_GUIDE.M with the given input arguments.
%
%      DOBOT_GUIDE('Property','Value',...) creates a new DOBOT_GUIDE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Dobot_guide_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      estop.  All inputs are passed to Dobot_guide_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Dobot_guide

% Last Modified by GUIDE v2.5 17-May-2021 02:10:30

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Dobot_guide_OpeningFcn, ...
                   'gui_OutputFcn',  @Dobot_guide_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before Dobot_guide is made visible.
function Dobot_guide_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Dobot_guide (see VARARGIN)

% Choose default command line output for Dobot_guide
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using Dobot_guide.
if strcmp(get(hObject,'Visible'),'off')
    plot(rand(1));
end

% UIWAIT makes Dobot_guide wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Dobot_guide_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes1);
dobot = Dobot();

%Plot environment
hold on;
PlaceObject('Table.ply',[0,0,0]);
PlaceObject('Fence.ply',[0,0,-1]);
PlaceObject('Human.ply',[0,0,-1]);
PlaceObject('polesy.ply',[0,0,-1]);
PlaceObject('sponge.ply',[-0.0163,   -0.2991,    0.03]);
PlaceObject('EStop.ply',[-0.3,-0.3,0]);
PlaceObject('WarningSign.ply',[0,0,-1]);
PlaceObject('Siren.ply',[-0.1,-1,-1.5]);
% Base
surf([-5,-5;5,5],[-5,5;-5,5],[-1,-1;-1,-1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[-1,-1;1000,1000],'CData',imread('sky.jpg'),'FaceColor','texturemap');
surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[1000,1000;-1,-1],'CData',imread('sky.jpg'),'FaceColor','texturemap');
% surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[-1,1000;-1,1000],'CData',imread('sky.jpg'),'FaceColor','texturemap');
surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[1000,-1;1000,-1],'CData',imread('sky.jpg'),'FaceColor','texturemap');


data = guidata(hObject);
data.model = dobot.model;
guidata(hObject,data);


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) + 0.01;
newQ = handles.model.ikcon(tr,q);
current_x_data = num2str(tr(1,4));
current_y_data = num2str(tr(2,4));
current_z_data = num2str(tr(3,4));
set(handles.current_x,'String',current_x_data);
set(handles.current_y,'String',current_y_data);
set(handles.current_z,'String',current_z_data);
handles.model.animate(newQ);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) - 0.01;
newQ = handles.model.ikcon(tr,q);
current_x_data = num2str(tr(1,4));
current_y_data = num2str(tr(2,4));
current_z_data = num2str(tr(3,4));
set(handles.current_x,'String',current_x_data);
set(handles.current_y,'String',current_y_data);
set(handles.current_z,'String',current_z_data);
handles.model.animate(newQ);


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(2,4) = tr(2,4) + 0.01;
newQ = handles.model.ikcon(tr,q);
current_x_data = num2str(tr(1,4));
current_y_data = num2str(tr(2,4));
current_z_data = num2str(tr(3,4));
set(handles.current_x,'String',current_x_data);
set(handles.current_y,'String',current_y_data);
set(handles.current_z,'String',current_z_data);
handles.model.animate(newQ);


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(2,4) = tr(2,4) - 0.01;
newQ = handles.model.ikcon(tr,q);
current_x_data = num2str(tr(1,4));
current_y_data = num2str(tr(2,4));
current_z_data = num2str(tr(3,4));
set(handles.current_x,'String',current_x_data);
set(handles.current_y,'String',current_y_data);
set(handles.current_z,'String',current_z_data);
handles.model.animate(newQ);

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) + 0.01;
newQ = handles.model.ikcon(tr,q);
current_x_data = num2str(tr(1,4));
current_y_data = num2str(tr(2,4));
current_z_data = num2str(tr(3,4));
set(handles.current_x,'String',current_x_data);
set(handles.current_y,'String',current_y_data);
set(handles.current_z,'String',current_z_data);
handles.model.animate(newQ);

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) - 0.01;
newQ = handles.model.ikcon(tr,q);
current_x_data = num2str(tr(1,4));
current_y_data = num2str(tr(2,4));
current_z_data = num2str(tr(3,4));
set(handles.current_x,'String',current_x_data);
set(handles.current_y,'String',current_y_data);
set(handles.current_z,'String',current_z_data);
handles.model.animate(newQ);


% --- Executes on slider movement.
function joint1_slider_Callback(hObject, eventdata, handles)
% hObject    handle to joint1_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

q = handles.model.getpos;
current_q1 = get(hObject,'Value');
q(1) = current_q1;
handles.model.animate(q);
set(handles.joint1,'String',num2str(rad2deg(q(1))));
set(handles.joint2,'String',num2str(rad2deg(q(2))));
set(handles.joint3,'String',num2str(rad2deg(q(3))));
set(handles.joint4,'String',num2str(rad2deg(q(4))));
set(handles.joint5,'String',num2str(rad2deg(q(5))));
tr = handles.model.fkine(q);
current_x_data = num2str(tr(1,4));
current_y_data = num2str(tr(2,4));
current_z_data = num2str(tr(3,4));
set(handles.current_x,'String',current_x_data);
set(handles.current_y,'String',current_y_data);
set(handles.current_z,'String',current_z_data);
% --- Executes during object creation, after setting all properties.
function joint1_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint1_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function joint1_Callback(hObject, eventdata, handles)
% hObject    handle to joint1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of joint1 as text
%        str2double(get(hObject,'String')) returns contents of joint1 as a double


% --- Executes during object creation, after setting all properties.
function joint1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function joint2_slider_Callback(hObject, eventdata, handles)
% hObject    handle to joint2_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
q = handles.model.getpos;
current_q2 = get(hObject,'Value');
q(2) = current_q2;
q(4) = pi - q(2) - q(3);
handles.model.animate(q);
set(handles.joint1,'String',num2str(rad2deg(q(1))));
set(handles.joint2,'String',num2str(rad2deg(q(2))));
set(handles.joint3,'String',num2str(rad2deg(q(3))));
set(handles.joint4,'String',num2str(rad2deg(q(4))));
set(handles.joint5,'String',num2str(rad2deg(q(5))));
tr = handles.model.fkine(q);
current_x_data = num2str(tr(1,4));
current_y_data = num2str(tr(2,4));
current_z_data = num2str(tr(3,4));
set(handles.current_x,'String',current_x_data);
set(handles.current_y,'String',current_y_data);
set(handles.current_z,'String',current_z_data);

% --- Executes during object creation, after setting all properties.
function joint2_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint2_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function joint3_slider_Callback(hObject, eventdata, handles)
% hObject    handle to joint3_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
q = handles.model.getpos;
current_q3 = get(hObject,'Value');
q(3) = current_q3;
q(4) = pi - q(2) - q(3);
handles.model.animate(q);
set(handles.joint1,'String',num2str(rad2deg(q(1))));
set(handles.joint2,'String',num2str(rad2deg(q(2))));
set(handles.joint3,'String',num2str(rad2deg(q(3))));
set(handles.joint4,'String',num2str(rad2deg(q(4))));
set(handles.joint5,'String',num2str(rad2deg(q(5))));
tr = handles.model.fkine(q);
current_x_data = num2str(tr(1,4));
current_y_data = num2str(tr(2,4));
current_z_data = num2str(tr(3,4));
set(handles.current_x,'String',current_x_data);
set(handles.current_y,'String',current_y_data);
set(handles.current_z,'String',current_z_data);

% --- Executes during object creation, after setting all properties.
function joint3_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint3_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function joint4_slider_Callback(hObject, eventdata, handles)
% hObject    handle to joint4_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
q = handles.model.getpos;

q(4) = pi - q(2) - q(3);
handles.model.animate(q);
set(handles.joint1,'String',num2str(rad2deg(q(1))));
set(handles.joint2,'String',num2str(rad2deg(q(2))));
set(handles.joint3,'String',num2str(rad2deg(q(3))));
set(handles.joint4,'String',num2str(rad2deg(q(4))));
set(handles.joint5,'String',num2str(rad2deg(q(5))));
tr = handles.model.fkine(q);
current_x_data = num2str(tr(1,4));
current_y_data = num2str(tr(2,4));
current_z_data = num2str(tr(3,4));
set(handles.current_x,'String',current_x_data);
set(handles.current_y,'String',current_y_data);
set(handles.current_z,'String',current_z_data);

% --- Executes during object creation, after setting all properties.
function joint4_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint4_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function joint5_slider_Callback(hObject, eventdata, handles)
% hObject    handle to joint5_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
q = handles.model.getpos;
TR = handles.model.fkine(q);
rpy = tr2rpy(TR);
q(5) = rpy(3) - q(1);
handles.model.animate(q);
set(handles.joint1,'String',num2str(rad2deg(q(1))));
set(handles.joint2,'String',num2str(rad2deg(q(2))));
set(handles.joint3,'String',num2str(rad2deg(q(3))));
set(handles.joint4,'String',num2str(rad2deg(q(4))));
set(handles.joint5,'String',num2str(rad2deg(q(5))));
tr = handles.model.fkine(q);
current_x_data = num2str(tr(1,4));
current_y_data = num2str(tr(2,4));
current_z_data = num2str(tr(3,4));
set(handles.current_x,'String',current_x_data);
set(handles.current_y,'String',current_y_data);
set(handles.current_z,'String',current_z_data);
% --- Executes during object creation, after setting all properties.
function joint5_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint5_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function joint2_Callback(hObject, eventdata, handles)
% hObject    handle to joint2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of joint2 as text
%        str2double(get(hObject,'String')) returns contents of joint2 as a double


% --- Executes during object creation, after setting all properties.
function joint2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function joint3_Callback(hObject, eventdata, handles)
% hObject    handle to joint3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of joint3 as text
%        str2double(get(hObject,'String')) returns contents of joint3 as a double


% --- Executes during object creation, after setting all properties.
function joint3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function joint4_Callback(hObject, eventdata, handles)
% hObject    handle to joint4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of joint4 as text
%        str2double(get(hObject,'String')) returns contents of joint4 as a double

% --- Executes during object creation, after setting all properties.
function joint4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function joint5_Callback(hObject, eventdata, handles)
% hObject    handle to joint5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of joint5 as text
%        str2double(get(hObject,'String')) returns contents of joint5 as a double


% --- Executes during object creation, after setting all properties.
function joint5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in eStop.
function eStop_Callback(hObject, eventdata, handles)
% hObject    handle to eStop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.eStopState = 1
guidata(hObject,handles)
disp('E-STOP ACTIVATED');
stopMsg = 'E-STOP ACTIVATED';
set(handles.robot_status,'String',stopMsg);
uiwait(Dobot_guide);
% --- Executes on button press in Continue.
function Continue_Callback(hObject, eventdata, handles)
% hObject    handle to Continue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);
if (handles.eStopState == 2)
    handles.eStopState = 0
    guidata(hObject,handles);
    disp('ROBOT OPERATION RESUMED');
     stopMsg = 'ROBOT OPERATION RESUMED';
set(handles.robot_status,'String',stopMsg);
    uiresume(Dobot_guide);
elseif(handles.eStopState == 0)
    disp('ROBOT IS OPERATING');
    stopMsg = 'ROBOT IS OPERATING';
set(handles.robot_status,'String',stopMsg);
else
    disp('RELEASE E-STOP FIRST');
     stopMsg = 'RELEASE E-STOP FIRST';
set(handles.robot_status,'String',stopMsg);
end

% --- Executes on button press in wipeAll.
function wipeAll_Callback(hObject, eventdata, handles)
% hObject    handle to wipeAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);
end_effector_rotation = [0,0,0];

% Cordinates: XYZ
waypointCoords{1} = [0.2067         0    0.1350];        % q = zeros(1,4)
waypointCoords{2} = [0.1710   -0.1177    0.1376];
waypointCoords{3} = [0.0882   -0.1875    0.1383];
waypointCoords{4} = [-0.0078   -0.2064    0.1379];
waypointCoords{5} = [-0.0145   -0.2993    0.09];       % right above sponge
waypointCoords{6} = [-0.0163   -0.2991    0.03];       % ready to close gripper
waypointCoords{7} = [0.2057   -0.2312    0.037]; 
waypointCoords{8} = [0.2057   0.2312    0.037];

% Poses: TR
for i=1:length(waypointCoords)
    waypointPoses{i} = eul2tr(end_effector_rotation) * transl(waypointCoords{i}(1),waypointCoords{i}(2),waypointCoords{i}(3));
end
steps = 20;

wipeSequence = [1,4,6,4,1,7,8,7,8];

for k=1:length(wipeSequence)
    
    T1 = waypointPoses{wipeSequence(k)};
    if(k == length(wipeSequence))
        T2 = waypointPoses{1};
    else
        T2 = waypointPoses{wipeSequence(k+1)};
    end
    
    
    q1 = IKdobot_inputTransform(T1);
    q2 = IKdobot_inputTransform(T2);
    qMatrix = jtraj(q1,q2,steps);
    result = true(steps,1);     %create a logical vector
    robotInCollision = false;
    int_collision = false;
% go through each step of the trajectory and check the result to see if it is in collision
for a = 1: steps
    result(a) = false;
    q = qMatrix(a,:);
    tr = zeros(4,4,handles.model.n+1);
    tr(:,:,1) = handles.model.base;
    L = handles.model.links;
    for b = 1:handles.model.n
         tr(:,:,b+1) = tr(:,:,b) * trotz(q(b)+L(b).offset) * transl(0,0,L(b).d) * transl(L(b).a,0,0) * trotx(L(b).alpha);
    end
    % Get the transform of every joint (i.e. start and end of every link)
    if get(handles.simu_colli,'Value')==1
        for i = 1:size(tr,3)-1
             
                
                for faceIndex = 1:size(handles.cube.faces,1)
                    vertOnPlane = handles.cube.vertex(handles.cube.faces(faceIndex,1)',:);
                    [intersectP,check] = LinePlaneIntersection(handles.cube.faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                    if check==1 && IsIntersectionPointInsideTriangle(intersectP,handles.cube.vertex(handles.cube.faces(faceIndex,:)',:))
                        plot3(intersectP(1),intersectP(2),intersectP(3),'*g');
    %                     display('intersection');
                        result = true;      % set step of logical vector true if incollision
                        robotInCollision = true;
                    end
                end
%              elseif get(handles.simu_colli,'Value')==0
%                  
%                  continue;
                 
             end   
        end
        if get(handles.simu_env,'Value')== 1 && get(handles.simu_inter,'Value') == 1
        for faceIndex_int = 1:size(handles.inter.faces_int,1)-1
        for faceIndex_cage = 1:size(handles.cage.faces_cage,1)
 
                vertOnPlane_cage = handles.cage.vertex_cage(handles.cage.faces_cage(faceIndex_cage,1)',:);      % simulated cage cube
                vertOnPlane_int_1 = handles.inter.vertex_int(handles.inter.faces_int(faceIndex_int,1)',:);         % simulated interupt object cube
                vertOnPlane_int_2 = handles.inter.vertex_int(handles.inter.faces_int(faceIndex_int,2)',:);
        
                [intersectP,check] = LinePlaneIntersection(handles.cage.faceNormals_cage(faceIndex_cage,:),vertOnPlane_cage,vertOnPlane_int_1,vertOnPlane_int_2); 
                if check==1 && IsIntersectionPointInsideTriangle(intersectP,handles.cage.vertex_cage(handles.cage.faces_cage(faceIndex_cage,:)',:))
                    result = true;
                    int_collision = true;
                    
                end
                end
        end
        end
     if robotInCollision == true || int_collision ==true
        disp('Collision Detected or Environment is unsafe');
        stopMsg = 'Collision Detected or Environment is unsafe';
        set(handles.robot_status,'String',stopMsg);
        return;
     else
        handles.model.animate(qMatrix(a,:));     
        disp('Robot is operating safely'); 
        stopMsg = 'Robot is operating safely';
        set(handles.robot_status,'String',stopMsg);
     end

end
  
end
  
         
   
    
% --- Executes on button press in wipe_area_1.
function wipe_area_1_Callback(hObject, eventdata, handles)
% hObject    handle to wipe_area_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

steps = 50;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
qMatrix = zeros(steps,5);

wipeLocation = [0.225 0];
wipeRadius = [0.03,0.1];
wipeHeight = 0.037;

for i=1:steps
    x(1,i) = wipeLocation(1) + wipeRadius(1)*sin(delta*i);
    x(2,i) = wipeLocation(2) + wipeRadius(2)*cos(delta*i);
    x(3,i) = wipeHeight    ;                 
    theta(1,i) = 0;                     % Roll angle 
    theta(2,i) = 0;                     % Pitch angle
    theta(3,i) = 0;                     % Yaw angle
    T = [rpy2r(theta(1,i),theta(2,i),theta(3,i)) x(:,i);    % create transformation of first point and angle
        zeros(1,3)  1 ];
    qMatrix(i,:) = IKdobot_inputTransform(T);    
end
 result = true(steps,1);     %create a logical vector
    robotInCollision = false;
    int_collision = false;
% go through each step of the trajectory and check the result to see if it is in collision
for a = 1: steps
    result(a) = false;
    q = qMatrix(a,:);
    tr = zeros(4,4,handles.model.n+1);
    tr(:,:,1) = handles.model.base;
    L = handles.model.links;
    for b = 1:handles.model.n
         tr(:,:,b+1) = tr(:,:,b) * trotz(q(b)+L(b).offset) * transl(0,0,L(b).d) * transl(L(b).a,0,0) * trotx(L(b).alpha);
    end
    % Get the transform of every joint (i.e. start and end of every link)
        for i = 1:size(tr,3)-1
             if get(handles.simu_colli,'Value')==1
                 guidata(hObject,handles);
                for faceIndex = 1:size(handles.cube.faces,1)
                    vertOnPlane = handles.cube.vertex(handles.cube.faces(faceIndex,1)',:);
                    [intersectP,check] = LinePlaneIntersection(handles.cube.faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                    if check==1 && IsIntersectionPointInsideTriangle(intersectP,handles.cube.vertex(handles.cube.faces(faceIndex,:)',:))
                        plot3(intersectP(1),intersectP(2),intersectP(3),'*g');
    %                     display('intersection');
                        result = true;      % set step of logical vector true if incollision
                        robotInCollision = true;
                    end
                end
%              elseif get(handles.simu_colli,'Value')==0
%                  
%                  continue;
                 
             end   
        end
        if get(handles.simu_env,'Value')== 1 && get(handles.simu_inter,'Value') == 1
        for faceIndex_int = 1:size(handles.inter.faces_int,1)-1
        for faceIndex_cage = 1:size(handles.cage.faces_cage,1)
 
                vertOnPlane_cage = handles.cage.vertex_cage(handles.cage.faces_cage(faceIndex_cage,1)',:);      % simulated cage cube
                vertOnPlane_int_1 = handles.inter.vertex_int(handles.inter.faces_int(faceIndex_int,1)',:);         % simulated interupt object cube
                vertOnPlane_int_2 = handles.inter.vertex_int(handles.inter.faces_int(faceIndex_int,2)',:);
        
                [intersectP,check] = LinePlaneIntersection(handles.cage.faceNormals_cage(faceIndex_cage,:),vertOnPlane_cage,vertOnPlane_int_1,vertOnPlane_int_2); 
                if check==1 && IsIntersectionPointInsideTriangle(intersectP,handles.cage.vertex_cage(handles.cage.faces_cage(faceIndex_cage,:)',:))
                    result = true;
                    int_collision = true;
                    
                end
                end
        end
        end
     if robotInCollision == true || int_collision ==true
        disp('Collision Detected or Environment is unsafe');
        stopMsg = 'Collision Detected or Environment is unsafe';
        set(handles.robot_status,'String',stopMsg);
        return;
     else
        handles.model.animate(qMatrix(a,:));     
        disp('Robot is operating safely');  
        stopMsg = 'Robot is operating safely';
        set(handles.robot_status,'String',stopMsg);
     end

end
  



% --- Executes on button press in wipe_area_2.
function wipe_area_2_Callback(hObject, eventdata, handles)
% hObject    handle to wipe_area_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

steps = 50;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
qMatrix = zeros(steps,5);

wipeLocation = [0 0.2];
wipeRadius = [0.1,0.03];
wipeHeight = 0.037;

for i=1:steps
    x(1,i) = wipeLocation(1) + wipeRadius(1)*sin(delta*i);
    x(2,i) = wipeLocation(2) + wipeRadius(2)*cos(delta*i);
    x(3,i) = wipeHeight  ;                   
    theta(1,i) = 0;                     % Roll angle 
    theta(2,i) = 0;                     % Pitch angle
    theta(3,i) = 0;                     % Yaw angle
    T = [rpy2r(theta(1,i),theta(2,i),theta(3,i)) x(:,i);    % create transformation of first point and angle
        zeros(1,3)  1 ];
    qMatrix(i,:) = IKdobot_inputTransform(T);
%     qMatrix(i,:) = dobot.model.ikcon(T)
end

    robotInCollision = false;
    int_collision = false;
% go through each step of the trajectory and check the result to see if it is in collision
for a = 1: steps
    result(a) = false;
    q = qMatrix(a,:);
    tr = zeros(4,4,handles.model.n+1);
    tr(:,:,1) = handles.model.base;
    L = handles.model.links;
    for b = 1:handles.model.n
         tr(:,:,b+1) = tr(:,:,b) * trotz(q(b)+L(b).offset) * transl(0,0,L(b).d) * transl(L(b).a,0,0) * trotx(L(b).alpha);
    end
    % Get the transform of every joint (i.e. start and end of every link)
        for i = 1:size(tr,3)-1
             if get(handles.simu_colli,'Value')==1
                 guidata(hObject,handles);
                for faceIndex = 1:size(handles.cube.faces,1)
                    vertOnPlane = handles.cube.vertex(handles.cube.faces(faceIndex,1)',:);
                    [intersectP,check] = LinePlaneIntersection(handles.cube.faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                    if check==1 && IsIntersectionPointInsideTriangle(intersectP,handles.cube.vertex(handles.cube.faces(faceIndex,:)',:))
                        plot3(intersectP(1),intersectP(2),intersectP(3),'*g');
    %                     display('intersection');
                        result = true;      % set step of logical vector true if incollision
                        robotInCollision = true;
                    end
                end
%              elseif get(handles.simu_colli,'Value')==0
%                  
%                  continue;
                 
             end   
        end
        if get(handles.simu_env,'Value')== 1 && get(handles.simu_inter,'Value') == 1
        for faceIndex_int = 1:size(handles.inter.faces_int,1)-1
        for faceIndex_cage = 1:size(handles.cage.faces_cage,1)
 
                vertOnPlane_cage = handles.cage.vertex_cage(handles.cage.faces_cage(faceIndex_cage,1)',:);      % simulated cage cube
                vertOnPlane_int_1 = handles.inter.vertex_int(handles.inter.faces_int(faceIndex_int,1)',:);         % simulated interupt object cube
                vertOnPlane_int_2 = handles.inter.vertex_int(handles.inter.faces_int(faceIndex_int,2)',:);
        
                [intersectP,check] = LinePlaneIntersection(handles.cage.faceNormals_cage(faceIndex_cage,:),vertOnPlane_cage,vertOnPlane_int_1,vertOnPlane_int_2); 
                if check==1 && IsIntersectionPointInsideTriangle(intersectP,handles.cage.vertex_cage(handles.cage.faces_cage(faceIndex_cage,:)',:))
                    result = true;
                    int_collision = true;
                    
                end
                end
        end
        end
     if robotInCollision == true || int_collision ==true
        disp('Collision Detected or Environment is unsafe');
         stopMsg = 'Collision Detected or Environment is unsafe';
        set(handles.robot_status,'String',stopMsg);
        return;
     else
        handles.model.animate(qMatrix(a,:));     
        disp('Robot is operating safely');  
        stopMsg = 'Robot is operating safely';
        set(handles.robot_status,'String',stopMsg);
     end

end



% --- Executes on button press in pick_up_spone.
function pick_up_spone_Callback(hObject, eventdata, handles)
% hObject    handle to pick_up_spone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
guidata(hObject,handles);


end_effector_rotation = [0,0,0];

% Cordinates: XYZ
waypointCoords{1} = [0.2067         0    0.1350];        % q = zeros(1,4)
waypointCoords{2} = [0.1710   -0.1177    0.1376];
waypointCoords{3} = [0.0882   -0.1875    0.1383];
waypointCoords{4} = [-0.0078   -0.2064    0.1379];
waypointCoords{5} = [-0.0145   -0.2993    0.09];       % right above sponge
waypointCoords{6} = [-0.0163   -0.2991    0.03];       % ready to close gripper
waypointCoords{7} = [0.2057   -0.2312    0.037]; 
waypointCoords{8} = [0.2057   0.2312    0.037];

% Poses: TR
for i=1:length(waypointCoords)
    waypointPoses{i} = eul2tr(end_effector_rotation) * transl(waypointCoords{i}(1),waypointCoords{i}(2),waypointCoords{i}(3));
end

steps = 20;

% define the transformations for two poses
T1 = waypointPoses{1};      % first pose
T2 = waypointPoses{6};      % second pose

% use custom IK to solve for joint angles of each pose
q1 = IKdobot_inputTransform(T1);         % sovle for joint angles
q2 = IKdobot_inputTransform(T2);         % sove for joint angles
handles.model.animate(q1);  % plot first pose
pause(2);

% Use joint interpolation to move between the two poses and plot path
qMatrix = jtraj(q1,q2,steps);

% Find all the joint states in trajectory that are in collision
    robotInCollision = false;
    int_collision = false;
% go through each step of the trajectory and check the result to see if it is in collision
for a = 1: steps
    result(a) = false;
    q = qMatrix(a,:);
    tr = zeros(4,4,handles.model.n+1);
    tr(:,:,1) = handles.model.base;
    L = handles.model.links;
    for b = 1:handles.model.n
         tr(:,:,b+1) = tr(:,:,b) * trotz(q(b)+L(b).offset) * transl(0,0,L(b).d) * transl(L(b).a,0,0) * trotx(L(b).alpha);
    end
    % Get the transform of every joint (i.e. start and end of every link)
        for i = 1:size(tr,3)-1
             if get(handles.simu_colli,'Value')==1
                 guidata(hObject,handles);
                for faceIndex = 1:size(handles.cube.faces,1)
                    vertOnPlane = handles.cube.vertex(handles.cube.faces(faceIndex,1)',:);
                    [intersectP,check] = LinePlaneIntersection(handles.cube.faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                    if check==1 && IsIntersectionPointInsideTriangle(intersectP,handles.cube.vertex(handles.cube.faces(faceIndex,:)',:))
                        plot3(intersectP(1),intersectP(2),intersectP(3),'*g');
    %                     display('intersection');
                        result = true;      % set step of logical vector true if incollision
                        robotInCollision = true;
                    end
                end
%              elseif get(handles.simu_colli,'Value')==0
%                  
%                  continue;
                 
             end   
        end
        if get(handles.simu_env,'Value')== 1 && get(handles.simu_inter,'Value') == 1
        for faceIndex_int = 1:size(handles.inter.faces_int,1)-1
        for faceIndex_cage = 1:size(handles.cage.faces_cage,1)
 
                vertOnPlane_cage = handles.cage.vertex_cage(handles.cage.faces_cage(faceIndex_cage,1)',:);      % simulated cage cube
                vertOnPlane_int_1 = handles.inter.vertex_int(handles.inter.faces_int(faceIndex_int,1)',:);         % simulated interupt object cube
                vertOnPlane_int_2 = handles.inter.vertex_int(handles.inter.faces_int(faceIndex_int,2)',:);
        
                [intersectP,check] = LinePlaneIntersection(handles.cage.faceNormals_cage(faceIndex_cage,:),vertOnPlane_cage,vertOnPlane_int_1,vertOnPlane_int_2); 
                if check==1 && IsIntersectionPointInsideTriangle(intersectP,handles.cage.vertex_cage(handles.cage.faces_cage(faceIndex_cage,:)',:))
                    result = true;
                    int_collision = true;
                    
                end
                end
        end
        end
     if robotInCollision == true || int_collision ==true
        disp('Collision Detected or Environment is unsafe');
        stopMsg = 'Collision Detected or Environment is unsafe';
        set(handles.robot_status,'String',stopMsg);
        return;
     else
        handles.model.animate(qMatrix(a,:));     
        disp('Robot is operating safely'); 
        stopMsg = 'Robot is operating safely';
        set(handles.robot_status,'String',stopMsg);
     end

end


% --- Executes on button press in spawn_env_cage.




% --- Executes on button press in pushbutton29.
function pushbutton29_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton30.
function pushbutton30_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1


% --- Executes on button press in simu_colli.
function simu_colli_Callback(hObject, eventdata, handles)
% hObject    handle to simu_colli (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of simu_colli
cubeCenter = [0.175, -0.3, 0.25];
cubeSides = 0.3;
% cubeCenter = [1, 1, 1];
% cubeSides = 0.3;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(cubeCenter-cubeSides/2, cubeCenter+cubeSides/2,plotOptions);
handles.cube.vertex = vertex;
handles.cube.faces = faces;
handles.cube.faceNormals = faceNormals;
guidata(hObject,handles);
pause(0.5);

% --- Executes on button press in simu_env.
function simu_env_Callback(hObject, eventdata, handles)
% hObject    handle to simu_env (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of simu_env
cubeCenter_cage = [0, 0, 0];
cubeSides_cage = 2;

plotOptions.plotEdges = true;
[vertex_cage,faces_cage,faceNormals_cage] = RectangularPrism(cubeCenter_cage-cubeSides_cage/2, cubeCenter_cage+cubeSides_cage/2,plotOptions);
handles.cage.vertex_cage = vertex_cage;
handles.cage.faces_cage = faces_cage;
handles.cage.faceNormals_cage = faceNormals_cage;
guidata(hObject,handles);
if get(hObject,'Value')==1
    set(handles.robot_status,'String','Light curtain activated');
elseif get(hObject,'Value')==0
    set(handles.robot_status,'String','Light curtain deactivated');
end
% --- Executes on button press in simu_inter.
function simu_inter_Callback(hObject, eventdata, handles)
% hObject    handle to simu_inter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of simu_inter
cubeCenter_int = [1, 1, 0];
cubeSides_int = 0.5;

plotOptions.plotFaces = true;
[vertex_int,faces_int,faceNormals_int] = RectangularPrism(cubeCenter_int-cubeSides_int/2, cubeCenter_int+cubeSides_int/2,plotOptions);
handles.inter.vertex_int = vertex_int;
handles.inter.faces_int = faces_int;
handles.inter.faceNormals_int = faceNormals_int;
guidata(hObject,handles);

% --- Executes on button press in eStopRelease.
function eStopRelease_Callback(hObject, eventdata, handles)
% hObject    handle to eStopRelease (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);
if handles.eStopState == 1
    handles.eStopState = 2
    guidata(hObject,handles)
    disp('E-STOP RELEASED')
    stopMsg = 'E-STOP RELEASED';
    set(handles.robot_status,'String',stopMsg);
else
    disp('E-STOP ALREADY RELEASED')
    stopMsg = 'E-STOP ALREADY RELEASED';
set(handles.robot_status,'String',stopMsg);
end

 

% --- Executes on button press in Clear_button.
function Clear_button_Callback(hObject, eventdata, handles)
% hObject    handle to Clear_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla (handles.axes1,'reset');



function current_x_Callback(hObject, eventdata, handles)
% hObject    handle to current_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of current_x as text
%        str2double(get(hObject,'String')) returns contents of current_x as a double


% --- Executes during object creation, after setting all properties.
function current_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to current_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function current_y_Callback(hObject, eventdata, handles)
% hObject    handle to current_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of current_y as text
%        str2double(get(hObject,'String')) returns contents of current_y as a double


% --- Executes during object creation, after setting all properties.
function current_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to current_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function current_z_Callback(hObject, eventdata, handles)
% hObject    handle to current_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of current_z as text
%        str2double(get(hObject,'String')) returns contents of current_z as a double


% --- Executes during object creation, after setting all properties.
function current_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to current_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function robot_status_Callback(hObject, eventdata, handles)
% hObject    handle to robot_status (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of robot_status as text
%        str2double(get(hObject,'String')) returns contents of robot_status as a double


% --- Executes during object creation, after setting all properties.
function robot_status_CreateFcn(hObject, eventdata, handles)
% hObject    handle to robot_status (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in rmrc_button.
function rmrc_button_Callback(hObject, eventdata, handles)
% hObject    handle to rmrc_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Set parameters for the simulation
end_effector_rotation = [0,0,0];

% Cordinates: XYZ
waypointCoords{1} = [0.2067         0    0.1350];        % q = zeros(1,4)
waypointCoords{2} = [0.1710   -0.1177    0.1376];
waypointCoords{3} = [0.0882   -0.1875    0.1383];
waypointCoords{4} = [-0.0078   -0.2064    0.1379];
waypointCoords{5} = [-0.0145   -0.2993    0.09];       % right above sponge
waypointCoords{6} = [-0.0163   -0.2991    0.03];       % ready to close gripper
waypointCoords{7} = [0.2057   -0.2312    0.037]; 
waypointCoords{8} = [0.2057   0.2312    0.037];

% Poses: TR
for i=1:length(waypointCoords)
    waypointPoses{i} = eul2tr(end_effector_rotation) * transl(waypointCoords{i}(1),waypointCoords{i}(2),waypointCoords{i}(3));
end
startWaypointIndex = 8;
endWaypointIndex = 7;
x1 = waypointCoords{startWaypointIndex}';
x2 = waypointCoords{endWaypointIndex}';

% Set parameters for the simulation
t = 10;             % Total time (s)
deltaT = 0.1;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
% W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
W = diag([1 1 1 0.1 0.1]);

% Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,5);       % Array for joint angles
qdot = zeros(steps,5);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

% Set up trajectory, intial pose
s = lspb(0,1,steps);            % trapezoidal trajectory scalar
for i=1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;     % Points in x-y-z
    theta(1,i) = 0;                     % Roll angle 
    theta(2,i) = 0;                     % Pitch angle
    theta(3,i) = 0;                     % Yaw angle
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);    % create transformation of first point and angle
        zeros(1,3)  1 ];
% q0 = zeros(1,5);                                      % initial guess for joint angles
% qMatrix(1,:) = dobot.model.ikcon(T,q0);
qMatrix(1,:) = IKdobot_inputTransform(T);               % solve joint angles to achieve first waypoint                   

% Track trajectory with RMRC
for i=1:steps-1
    T = handles.model.fkine(qMatrix(i,:));                   % FK for pose at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                       % position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1)); % next RPY angles then convert to rotation matrix (desired rotation)
    Ra = T(1:3,1:3);                                    % actual rotation matrix
    Rdot = (1/deltaT) * (Rd - Ra);                      % rotation matrix error
    S = Rdot * Ra';                                     % skew symmetrix matrix to extract linear and angular velocity
    linear_velocity = (1/deltaT) * deltaX;
%     angular_velocity = [S(3,2); S(1,3); S(2,1)];
    angular_velocity = [S(3,2); S(1,3)];
    deltaTheta = tr2rpy(Rd * Ra');
    xdot = W*[linear_velocity;angular_velocity];        % calculate end-effector velocity to reach next waypoint
    J = handles.model.jacob0(qMatrix(i,:));             % get jacobian at current joint state
    J = J(1:5,:);           % take first 5 rows to make square matrix
    mom(i) = real(sqrt(det(J*J')));
    if mom(i) < epsilon                       % if manipulability is less than given threshold
        lambda = (1 - mom(i)/epsilon)*5E-2;
    else
        lamda = 0;
    end
    invJ = inv(J'*J + lambda *eye(5))*J';      % DLS inverse
    qdot(i,:) = (invJ * xdot)';                 % solve the RMRC equation
    
    for j = 1:5                     % Loop through joints 1 to 5
        % If next joint angle is lower than joint limit, stop the model
        if qMatrix(i,j) + deltaT*qdot(i,j) < handles.model.qlim(j,1)
            qdot(i,j) = 0;
        % If next joint angle is greater than joint limit, stop the model
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > handles.model.qlim(j,2)
            qdot(i,j) = 0;
        end
    end
    
%     update next joint state based on joint velocities
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);
    positionError(:,i) = x(:,i+1) - T(1:3,4);           % For plotting
    angleError(:,i) = deltaTheta;                       % For plotting

end

handles.model.plot(qMatrix,'trail','r-')
