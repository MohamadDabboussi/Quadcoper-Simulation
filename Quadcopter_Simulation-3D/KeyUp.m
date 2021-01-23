function KeyUp(~,key)
    switch key.Key
        case 'w'
            evalin('base','Data =Data-[0 0 Data(3)]');
        case 's'
             evalin('base','Data =Data- [0 0 Data(3)]');
       case 'a'
            evalin('base','Data =Data - [0 Data(2) 0]');
        case 'd'
             evalin('base','Data =Data - [0 Data(2) 0]');
       case 'c'
            evalin('base','Data =Data - [Data(1) 0 0]');
        case 'z'
             evalin('base','Data =Data - [Data(1) 0 0]');
    end
    pause(0.2);
end