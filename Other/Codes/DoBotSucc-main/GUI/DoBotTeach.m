function varargout = DoBotTeach(varargin)
% DOBOTTEACH MATLAB code for DoBotTeach.fig
%      DOBOTTEACH, by itself, creates a new DOBOTTEACH or raises the existing
%      singleton*.
%
%      H = DOBOTTEACH returns the handle to a new DOBOTTEACH or the handle to
%      the existing singleton*.
%
%      DOBOTTEACH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DOBOTTEACH.M with the given input arguments.
%
%      DOBOTTEACH('Property','Value',...) creates a new DOBOTTEACH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before DoBotTeach_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to DoBotTeach_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help DoBotTeach

% Last Modified by GUIDE v2.5 26-Apr-2022 10:46:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @DoBotTeach_OpeningFcn, ...
                   'gui_OutputFcn',  @DoBotTeach_OutputFcn, ...
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


% --- Executes just before DoBotTeach is made visible.
function DoBotTeach_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to DoBotTeach (see VARARGIN)

% Choose default command line output for DoBotTeach
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
robot = DoBot;
% robot.model.plot3d([0 0 0 0]);
robot.SetDoBot();
hold on
% global runonce;
% runonce = 1;

% UIWAIT makes DoBotTeach wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = DoBotTeach_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on slider movement.
function q1Slider_Callback(hObject, eventdata, handles)
% hObject    handle to q1Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

q1 = get(hObject,'Value');
set(handles.q1Text,'String',num2str(q1));
robot = DoBot;
q1pos = robot.model.getpos;
q1plot = [deg2rad(q1) q1pos(1,2:4)];
robot.model.plot3d(q1plot);


% --- Executes during object creation, after setting all properties.
function q1Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q2Slider_Callback(hObject, eventdata, handles)
% hObject    handle to q2Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
q2 = get(hObject,'Value');
robot = DoBot;
set(handles.q2Text,'String',num2str(q2));
q2 = q2*-1;
q2pos = robot.model.getpos

q2plot = [q2pos(1,1) deg2rad(q2) q2pos(1,3:4)];
robot.model.plot3d(q2plot);

% --- Executes during object creation, after setting all properties.
function q2Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q2Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q3Slider_Callback(hObject, eventdata, handles)
% hObject    handle to q3Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
q3 = get(hObject,'Value');
set(handles.q3Text,'String',num2str(q3));
robot = DoBot;
q3pos = robot.model.getpos;
q3plot = [q3pos(1,1:2) deg2rad(q3) q3pos(1,4)];
robot.model.plot3d(q3plot);

% --- Executes during object creation, after setting all properties.
function q3Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q3Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q4Slider_Callback(hObject, eventdata, handles)
% hObject    handle to q4Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
q4 = get(hObject,'Value');
set(handles.q4Text,'String',num2str(q4));
robot = DoBot;
q4 = q4 * -1
q4pos = robot.model.getpos;
q4plot = [q4pos(1,1:3) deg2rad(q4)];
robot.model.plot3d(q4plot);

% --- Executes during object creation, after setting all properties.
function q4Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q4Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function q1Text_Callback(hObject, eventdata, handles)
% hObject    handle to q1Text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1Text as text
%        str2double(get(hObject,'String')) returns contents of q1Text as a double
q1 = str2double(get(hObject,'String'));
set(handles.q1Slider,'Value',q1);
robot = DoBot;
q1pos = robot.model.getpos;
q1plot = [deg2rad(q1) q1pos(1,2:4)];
robot.model.plot3d(q1plot);
% --- Executes during object creation, after setting all properties.
function q1Text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1Text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
% set(handles.q1Text,'String',num2str(handles.q1Slider,'Value'));
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q2Text_Callback(hObject, eventdata, handles)
% hObject    handle to q2Text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q2Text as text
%        str2double(get(hObject,'String')) returns contents of q2Text as a double
q2 = str2double(get(hObject,'String'));
set(handles.q2Slider,'Value',q2);
robot = DoBot;
q2pos = robot.model.getpos;
q2plot = [q2pos(1,1) deg2rad(q2) q2pos(1,3:4)];
robot.model.plot3d(q2plot);

% --- Executes during object creation, after setting all properties.
function q2Text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q2Text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q3Text_Callback(hObject, eventdata, handles)
% hObject    handle to q3Text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q3Text as text
%        str2double(get(hObject,'String')) returns contents of q3Text as a double
q3 = str2double(get(hObject,'String'));
set(handles.q3Slider,'Value',q3);
robot = DoBot;
q3pos = robot.model.getpos;
q3plot = [q3pos(1,1:2) deg2rad(q3) q3pos(1,4)];
robot.model.plot3d(q3plot);

% --- Executes during object creation, after setting all properties.
function q3Text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q3Text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q4Text_Callback(hObject, eventdata, handles)
% hObject    handle to q4Text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q4Text as text
%        str2double(get(hObject,'String')) returns contents of q4Text as a double
q4 = str2double(get(hObject,'String'));
set(handles.q4Slider,'Value',q4);
robot = DoBot;
q4pos = robot.model.getpos;
q4plot = [q4pos(1,1:3) deg2rad(q4)];
robot.model.plot3d(q4plot);

% --- Executes during object creation, after setting all properties.
function q4Text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q4Text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function xslider_Callback(hObject, eventdata, handles)
% hObject    handle to xslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function xslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function yslider_Callback(hObject, eventdata, handles)
% hObject    handle to yslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function yslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function zslider_Callback(hObject, eventdata, handles)
% hObject    handle to zslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function zslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function xtext_Callback(hObject, eventdata, handles)
% hObject    handle to xtext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xtext as text
%        str2double(get(hObject,'String')) returns contents of xtext as a double


% --- Executes during object creation, after setting all properties.
function xtext_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xtext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ytext_Callback(hObject, eventdata, handles)
% hObject    handle to ytext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ytext as text
%        str2double(get(hObject,'String')) returns contents of ytext as a double


% --- Executes during object creation, after setting all properties.
function ytext_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ytext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ztext_Callback(hObject, eventdata, handles)
% hObject    handle to ztext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ztext as text
%        str2double(get(hObject,'String')) returns contents of ztext as a double


% --- Executes during object creation, after setting all properties.
function ztext_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ztext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in TeachButton.
function TeachButton_Callback(hObject, eventdata, handles)
% hObject    handle to TeachButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
robot = DoBot;
Posx= str2double(handles.xtext.String);
Posy= str2double(handles.ytext.String);
Posz= str2double(handles.ztext.String);

Tlat = [1 0 0 Posx;
     0 1 0 Posy;
     0 0 1 Posz;
     0 0 0 1;];
%  runonce =0;
%  if runonce == 0
%  q1 = [0 0 0 0];
%  runonce = 1
%  end
q1 =robot.model.getpos;
q2 = robot.model.ikcon(Tlat,q1);
robot.model.plot3d(q2);
