classdef object_finder < handle
      methods(Static)
        function [midpt, th, best] = find_all_lines(rImage, err, num, ro)
            i = 1;
            finished = false;
            midpt = [];
            th = [];
            best = [i 0];
            best_err = 0;
            th_array = rImage.tArray;
            while(~finished)
                current_t = th_array(i);
                if(abs(ro - current_t) >= pi/4 && 360 - abs(ro-current_t) >= pi / 4)
                    continue
                end
                [erre, nume, the] = rImage.findLineCandiate(i, 15);
                if erre < err && nume >= num
                    midpt = [midpt, i];
                    th = [th the];
                    if nume > best_err
                        best_err = nume;
                        best = [i, the];
                    end
                end
                i = rImage.inc(i);
                
                if(i == 1)
                    finished = true;
                end
            end
        end
        
        function [err, num, th] = findLineCandiate(rImage, middle, maxLen)
            left_end = middle;
            right_end = middle;
            counter = 1;
            valid = true;
            start_range = rImage.rArray(middle);
            err = 0;
            num = 0;
            th = 0;
            dist = 0;
            while(dist < maxLen && counter < rImage.numPix -3 && valid)
                left_end = dec(rImage, left_end);
                right_end = inc(rImage, right_end);
                counter = counter + 2;
                if abs(rImage.rArray(left_end) - start_range) > 0.1
                    left_end = rImage.inc(left_end);
                    right_end = rImage.dec(right_end);
                    break;
                elseif abs(rImage.rArray(right_end) - start_range) > 0.1
                    left_end = rImage.inc(left_end);
                    right_end = rImage.dec(right_end);
                    break;
                else
                    x_left = rImage.xArray(left_end);
                    y_left = rImage.yArray(left_end);
                    x_right = rImage.xArray(right_end);
                    y_right = rImage.yArray(right_end);
                    dist = sqrt((x_left - x_right)^2 + (y_left-y_right)^2)*100;
                end
            end
            while(rImage.rArray(left_end) == 0)
                left_end = inc(rImage, left_end);
            end
            while(rImage.rArray(right_end) == 0)
                right_end = dec(rImage, right_end);
            end
            
            x_left = rImage.xArray(left_end);
            y_left = rImage.yArray(left_end);
            x_right = rImage.xArray(right_end);
            y_right = rImage.yArray(right_end);

            th = atan2(x_left - x_right,-y_left + y_right);
            dist = sqrt((x_left - x_right)^2 + (y_left-y_right)^2)*100;
            if(dist < 5 || dist > 15)
                err = 10000;
            else
                m = (y_right - y_left)/ (x_right - x_left);
                x_c = left_end;
                while(x_c ~= right_end)
                    if(rImage.rArray(x_c) ~= 0)
                        current_x = rImage.xArray(x_c);
                        current_y = rImage.yArray(x_c);
                        y_diff = m*(current_x - x_left) + y_left;
                        err = err + (y_diff - current_y)^2;
                        num = num+1;
                    end
                    x_c = inc(rImage, x_c);
                end
                if num == 0
                    err = 10000;
                else
                    err = err / num;
                end
            end
        end
      end
end