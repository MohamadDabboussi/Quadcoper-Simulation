%% Key functions
function KeyPress(~,key)
    switch key.Key
        case 'w'
            %evalin('base','Data = Data +[0 0 1]');
            if(evalin('base','Data(end,3)')<10)
            evalin('base','Data = cat(1,Data,Data(end,:)+[0 0 0.1])');
            end
        case 's'
             if(evalin('base','Data(end,3)')>0)
             evalin('base','Data = cat(1,Data,Data(end,:)+[0 0 -0.1])');
             end
       case 'a'
            evalin('base','Data = cat(1,Data,Data(end,:)+[0 0.1 0])');
        case 'd'
             evalin('base','Data = cat(1,Data,Data(end,:)+[0 -0.1 0])');
       case 'c'
            evalin('base','Data = cat(1,Data,Data(end,:)+[0.1 0 0])');
        case 'z'
             evalin('base','Data = cat(1,Data,Data(end,:)+[-0.1 0 0])');
        case 'q'
            evalin('base','finish = 1');
    end
    pause(0.02);
end