
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
%      stop.  All inputs are passed to Dobot_guide_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Dobot_guide

% Last Modified by GUIDE v2.5 16-May-2021 06:31:31

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
    plot(rand(5));
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
    L(1) = Link('d',0.138,  'a',0,      'alpha',-pi/2, 'offset',0);
    L(2) = Link('d',0,      'a',0.135,  'alpha',0,     'offset',-pi/2);
    L(3) = Link('d',0,      'a',0.147,  'alpha',0,     'offset',0);
    L(4) = Link('d',0,      'a',0.0597,  'alpha',pi/2,  'offset',-pi/2);     
    L(5) = Link('d',0,      'a',0,      'alpha',0,     'offset',0);

    % Joint Limits
    L(1).qlim = [-135 135]*pi/180;
    L(2).qlim = [5 80]*pi/180;
    L(3).qlim = [15 170]*pi/180;
    L(4).qlim = [-90 90]*pi/180;
    L(5).qlim = [-85 85]*pi/180;

    model = SerialLink(L,'name','Dobot');
    workspace = [-1 1 -1 1 -0.3 1];
    scale = 0.5;
    qhome = [0    0.1166    1.4480    1.5770         0];
    model.plot(qhome,'noarrow','workspace',workspace,'scale',scale);
    view([45 30]);
    data = guidata(hObject);
    data.model = model;
    guidata(hObject,data)


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
handles.model.animate(newQ);

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) + 0.01;
newQ = handles.model.ikcon(tr,q);
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
q = handles.model.getpos;
set(handles.joint1,'String',num2str(q(1)));

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


% --- Executes on button press in Stop.
function Stop_Callback(hObject, eventdata, handles)
% hObject    handle to Stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
uiwait(Dobot_guide);
% --- Executes on button press in Continue.
function Continue_Callback(hObject, eventdata, handles)
% hObject    handle to Continue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
uiresume(Dobot_guide);

% --- Executes on button press in wipeAll.
function wipeAll_Callback(hObject, eventdata, handles)
% hObject    handle to wipeAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

steps = 25;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
qMatrix = zeros(steps,5);

 

% Wiping locations
wipeHeight = 0.126;           % height of surface above base of dobot
% Wiping areas
wipeCenter{1} = [0.125, 0.2];
wipeCenter{2} = [0.225, 0];
wipeCenter{3} = [0.125, -0.2];

 

wipeRadiusIncrement = 0.03;
wipeRadiusMin = 0.03;
wipeRadiusMax = 0.09;

 

for a=1:length(wipeCenter)
    for wipeRadius=wipeRadiusMin:wipeRadiusIncrement:wipeRadiusMax     % wipe in circles of increasing size
        for i=1:steps
            x(1,i) = wipeCenter{a}(1) + wipeRadius*sin(delta*i);
            x(2,i) = wipeCenter{a}(2) + wipeRadius*cos(delta*i);
            x(3,i) = wipeHeight;                     
            theta(1,i) = 0;                     % Roll angle 
            theta(2,i) = 0;                     % Pitch angle
            theta(3,i) = 0;                     % Yaw angle
            T = [rpy2r(theta(1,i),theta(2,i),theta(3,i)) x(:,i);    % create transformation of first point and angle
                zeros(1,3)  1 ];
            qMatrix(i,:) = IKdobot_inputTransform(T);
%             qMatrix(i,:) = dobot.model.ikcon(T)
        end

 

        handles.model.plot(qMatrix,'trail','r-');
    end
end
            

  
         
   
        
        
        
