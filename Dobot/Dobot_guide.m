
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

% Last Modified by GUIDE v2.5 21-May-2021 13:04:50

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
workspace = [-2.5 2.5 -2.5 2.5 -2.2 2];
%         workspace = [-0.5 0.5 -0.5 0.5 -0.5 0.5];
        scale = 0.25;
        qhome = [0    0.1166    1.4480    1.5770         0];
dobot.model.plot(qhome,'workspace',workspace,'scale',scale,'noarrow')
%Plot environment
%  hold on;
% PlaceObject('Table.ply',[0,0,0]);
% PlaceObject('Fence.ply',[0,0,-1]);
% PlaceObject('Human.ply',[0,0,-1]);
% PlaceObject('polesy.ply',[0,0,-1]);
% PlaceObject('sponge.ply',[-0.0163,   -0.2991,    0.03]);
% PlaceObject('EStop.ply',[-0.3,-0.3,0]);
% PlaceObject('WarningSign.ply',[0,0,-1]);
% PlaceObject('Siren.ply',[-0.1,-1,-1.5]);
% % Base
% surf([-5,-5;5,5],[-5,5;-5,5],[-1,-1;-1,-1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
% surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[-1,-1;1000,1000],'CData',imread('sky.jpg'),'FaceColor','texturemap');
% surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[1000,1000;-1,-1],'CData',imread('sky.jpg'),'FaceColor','texturemap');
% % surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[-1,1000;-1,1000],'CData',imread('sky.jpg'),'FaceColor','texturemap');
% surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[1000,-1;1000,-1],'CData',imread('sky.jpg'),'FaceColor','texturemap');
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
handles.eStopState = 1;        %raising eStopState to 1 and create the handles.eStopState
guidata(hObject,handles);       %update handles data
disp('E-STOP ACTIVATED')
Msg = 'E-STOP ACTIVATED';
set(handles.robot_status,'String',Msg);
uiwait(Dobot_guide);           % stop Matlab execution in gui and wait for the uiresume
% --- Executes on button press in eStopRelease.
function eStopRelease_Callback(hObject, eventdata, handles)
% hObject    handle to eStopRelease (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);             % achieve handles data
if handles.eStopState == 1              % if eStopState = 1 ( eStop button pressed) 
    handles.eStopState = 2;              %  raise the eStopState to 2 to for Continue button condition
    guidata(hObject,handles);            % update handles data
    disp('E-STOP RELEASED')
    Msg = 'E-STOP RELEASED';
set(handles.robot_status,'String',Msg);
else
    disp('E-STOP ALREADY RELEASED')
    Msg = 'E-STOP ALREADY RELEASED';
set(handles.robot_status,'String',Msg);
end
% --- Executes on button press in Continue.
function Continue_Callback(hObject, eventdata, handles)
% hObject    handle to Continue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);
if (handles.eStopState == 2)              % if eStopState = 2 (Release eStop button has been pressed) 
    handles.eStopState = 0;               % set the eStopState to 0 and execute uiresume to continue matlab execution             
    guidata(hObject,handles);             % update handles data
    disp('ROBOT OPERATION RESUMED');
     Msg = 'ROBOT OPERATION RESUMED';           % display message for different eStopState
set(handles.robot_status,'String',Msg);
    uiresume(Dobot_guide);
elseif(handles.eStopState == 0)
    disp('ROBOT IS OPERATING');
    Msg = 'ROBOT IS OPERATING';
set(handles.robot_status,'String',Msg);
else
    disp('RELEASE E-STOP FIRST');            % ask the user to press release e stop
     Msg = 'RELEASE E-STOP FIRST';
set(handles.robot_status,'String',Msg);
end

% --- Executes on button press in wipeAll.
function wipeAll_Callback(hObject, eventdata, handles)
% hObject    handle to wipeAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);              % achieve handles data

[waypointCoords,waypointPoses,wipeSequence,steps] = GeneralWipe_data(50);

for k=1:length(wipeSequence)
    
    [qMatrix] = Waypoint_to_trajectory(waypointPoses,wipeSequence,k,steps);
   
    robotInCollision = false;
    interupt_collision = false;
% go through each step of the trajectory and check the result to see if it is in collision
for a = 1: steps
    q = qMatrix(a,:);
    tr = GetLinkPoses(handles.model.n,handles.model.base,handles.model.links,q);
   
    % Get the transform of every joint (i.e. start and end of every link)
    if get(handles.simu_colli,'Value')==1
            [collide_state] = Collision_checking(handles.cube.vertex,handles.cube.faces,handles.cube.faceNormals,tr);
            if collide_state == 1
                robotInCollision = true;
            end
        end
        if get(handles.simu_env,'Value')== 1 && get(handles.simu_inter,'Value') == 1
        [interupt_state] = Interuption_checking(handles.inter.vertex_int,handles.cage.vertex_cage,handles.inter.faces_int,handles.cage.faces_cage,handles.cage.faceNormals_cage);
        if interupt_state == 1
            interupt_collision = true;
        end
        end
     if robotInCollision == true || interupt_collision ==true
        disp('Collision Detected or Environment is unsafe');
        Msg = 'Collision Detected or Environment is unsafe';
        set(handles.robot_status,'String',Msg);
        return;
     else
        handles.model.animate(qMatrix(a,:));     
        disp('Robot is operating safely'); 
        Msg = 'Robot is operating safely';
        set(handles.robot_status,'String',Msg);
     end

end
  
end
  
         
   
    
% --- Executes on button press in wipe_area_1.
function wipe_area_1_Callback(hObject, eventdata, handles)
% hObject    handle to wipe_area_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject); 
wipeLocation1 = [0.225 0];
wipeRadius1 = [0.03,0.1];
wipeHeight1 = 0.037;

[qMatrix,steps] = Wipe_Eclipse(wipeLocation1,wipeRadius1,wipeHeight1,50);
 robotInCollision = false;
 interupt_collision = false;
% go through each step of the trajectory and check the result to see if it is in collision
for a = 1: steps
    q = qMatrix(a,:);
    tr = GetLinkPoses(handles.model.n,handles.model.base,handles.model.links,q);
    % Get the transform of every joint (i.e. start and end of every link)
        if get(handles.simu_colli,'Value')==1
            [collide_state] = Collision_checking(handles.cube.vertex,handles.cube.faces,handles.cube.faceNormals,tr);
            if collide_state == 1
                robotInCollision = true;
            end
        end

       if get(handles.simu_env,'Value')== 1 && get(handles.simu_inter,'Value') == 1
        [interupt_state] = Interuption_checking(handles.inter.vertex_int,handles.cage.vertex_cage,handles.inter.faces_int,handles.cage.faces_cage,handles.cage.faceNormals_cage);
        if interupt_state == 1
            interupt_collision = true;
        end
       end
     if robotInCollision == true || interupt_collision ==true
        Msg = 'Collision Detected or Environment is unsafe';
        set(handles.robot_status,'String',Msg);
        return;
     else
        handles.model.animate(qMatrix(a,:));      
        Msg = 'Robot is operating safely';
        set(handles.robot_status,'String',Msg);
     end

end
  

% --- Executes on button press in wipe_area_2.
function wipe_area_2_Callback(hObject, eventdata, handles)
% hObject    handle to wipe_area_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject); 
wipeLocation2 = [0 0.2];
wipeRadius2 = [0.1,0.03];
wipeHeight2 = 0.037;

[qMatrix,steps] = Wipe_Eclipse(wipeLocation2,wipeRadius2,wipeHeight2,50);
 robotInCollision = false;
 interupt_collision = false;
% go through each step of the trajectory and check the result to see if it is in collision
for a = 1: steps
    q = qMatrix(a,:);
    tr = GetLinkPoses(handles.model.n,handles.model.base,handles.model.links,q);
    % Get the transform of every joint (i.e. start and end of every link)
        if get(handles.simu_colli,'Value')==1
            [collide_state] = Collision_checking(handles.cube.vertex,handles.cube.faces,handles.cube.faceNormals,tr);
            if collide_state == 1
                robotInCollision = true;
            end
        end

       if get(handles.simu_env,'Value')== 1 && get(handles.simu_inter,'Value') == 1
        [interupt_state] = Interuption_checking(handles.inter.vertex_int,handles.cage.vertex_cage,handles.inter.faces_int,handles.cage.faces_cage,handles.cage.faceNormals_cage);
        if interupt_state == 1
            interupt_collision = true;
        end
       end
     if robotInCollision == true || interupt_collision ==true
        Msg = 'Collision Detected or Environment is unsafe';
        set(handles.robot_status,'String',Msg);
        return;
     else
        handles.model.animate(qMatrix(a,:));      
        Msg = 'Robot is operating safely';
        set(handles.robot_status,'String',Msg);
     end

end


% --- Executes on button press in pick_up_spone.
function pick_up_spone_Callback(hObject, eventdata, handles)
% hObject    handle to pick_up_spone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);              % achieve handles data
[waypointCoords,waypointPoses,wipeSequence,steps] = GeneralWipe_data(50);

% define the transformations for two poses
T1 = waypointPoses{1};      % first pose
T2 = waypointPoses{6};      % second pose

% use custom IK to solve for joint angles of each pose
q1 = IKdobot_inputTransform(T1);         % sovle for joint angles
q2 = IKdobot_inputTransform(T2);         % sove for joint angles
handles.model.animate(q1);  % plot first pose

% Use joint interpolation to move between the two poses and plot path
qMatrix = jtraj(q1,q2,steps);

% Find all the joint states in trajectory that are in collision
    robotInCollision = false;
    interupt_collision = false;
% go through each step of the trajectory and check the result to see if it is in collision
for a = 1: steps
   q = qMatrix(a,:);
    tr = GetLinkPoses(handles.model.n,handles.model.base,handles.model.links,q);
   
    % Get the transform of every joint (i.e. start and end of every link)
    if get(handles.simu_colli,'Value')==1
            [collide_state] = Collision_checking(handles.cube.vertex,handles.cube.faces,handles.cube.faceNormals,tr);
            if collide_state == 1
                robotInCollision = true;
            end
        end
        if get(handles.simu_env,'Value')== 1 && get(handles.simu_inter,'Value') == 1
        [interupt_state] = Interuption_checking(handles.inter.vertex_int,handles.cage.vertex_cage,handles.inter.faces_int,handles.cage.faces_cage,handles.cage.faceNormals_cage);
        if interupt_state == 1
            interupt_collision = true;
        end
        end
     if robotInCollision == true || interupt_collision ==true
        disp('Collision Detected or Environment is unsafe');
        Msg = 'Collision Detected or Environment is unsafe';
        set(handles.robot_status,'String',Msg);
        return;
     else
        handles.model.animate(qMatrix(a,:));     
        disp('Robot is operating safely'); 
        Msg = 'Robot is operating safely';
        set(handles.robot_status,'String',Msg);
     end

end
% --- Executes on button press in simu_colli.
function simu_colli_Callback(hObject, eventdata, handles)
% hObject    handle to simu_colli (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of simu_colli
cubeCenter = [0.175, -0.3, 0.25];
cubeSides = 0.3;
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
cubeCenter_int = [1, 1, 1];
cubeSides_int = 0.5;

plotOptions.plotFaces = true;
[vertex_int,faces_int,faceNormals_int] = RectangularPrism(cubeCenter_int-cubeSides_int/2, cubeCenter_int+cubeSides_int/2,plotOptions);
handles.inter.vertex_int = vertex_int;
handles.inter.faces_int = faces_int;
handles.inter.faceNormals_int = faceNormals_int;
guidata(hObject,handles);



 

% --- Executes on button press in Clear_button.
function Clear_button_Callback(hObject, eventdata, handles)
% hObject    handle to Clear_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla (handles.axes1,'reset');
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
[waypointCoords,waypointPoses,wipeSequence,steps] = GeneralWipe_data(50);
[qMatrix] = RMRC_motion(waypointCoords,8,7,handles.model,10,0.1,0.1);
handles.model.plot(qMatrix,'trail','r-')

