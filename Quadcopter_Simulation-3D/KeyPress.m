%% Key functions
function KeyPress(~,key)
    switch key.Key
        case 'w'
            evalin('base','Data =Data+[0 0 0.5]');
        case 's'
             evalin('base','Data =Data+ [0 0 -0.5]');
       case 'a'
            evalin('base','Data =Data+ [0 0.5 0]');
        case 'd'
             evalin('base','Data =Data+ [0 -0.5 0]');
       case 'c'
            evalin('base','Data =Data+ [0.5 0 0]');
        case 'z'
             evalin('base','Data =Data+ [-0.5 0 0]');
        case 'q'
            evalin('base','finish = 1');
    end
    pause(0.05);
end