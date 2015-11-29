classdef object_finder < handle
      methods(Static)
          
        function [midpt, th, best] = find_all_lines(rImage, err, num) %, ro)
            i = 1;
            finished = false;
            midpt = [];
            th = [];
            best = [i 0];
            best_err = 0;
            th_array = rImage.tArray;
            while(~finished)
                current_t = th_array(i);
%                 if(abs(ro - current_t) >= pi/4 && 360 - abs(ro-current_t) >= pi / 4)
%                     continue
%                 end
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
        
        function pt = find_line_ht(x_array, y_array, th_array, num)
            image_mat = uint8(zeros(4/0.01, 4/0.01));
%             x_array = rImage.xArray;
%             y_array = rImage.yArray;
%             num = rImage.numPix;
            best = [];
            for i = 1:num
                x = x_array(i)*100;
                x = round(x + 200);
                y = y_array(i)*100;
                y = round(200+y);
                image_mat(y, x) = 1;
            end
            [H, theta, rho] = hough(image_mat);
            P = houghpeaks(H, 5, 'threshold', 0.1*max(H(:)), 'NHoodSize', [21,21]);

            for i = 1:size(P, 1)
                cr = rho(P(i, 1));
                ct = deg2rad(theta(P(i,2)));
                temp = [];
                for j= 1:num
                    x = x_array(j)*100;
                    x = round(x + 200);
                    y = y_array(j)*100;
                    y = round(200+y);
                    guess = x * cos(ct) + y * sin(ct);
                    if(abs(guess - cr) <= 5)
                        temp = [temp; x_array(j),y_array(j), th_array(j)];
                    end
                end
                
                inliers = size(temp, 1);
                [v,max_coord] = max(temp(:, 1));
                [v,min_coord] = min(temp(:, 1));
                p1 = temp(max_coord, :);
                p2 = temp(min_coord, :);
                dist = (p1(1) - p2(1))^2 + (p1(2) - p2(2))^2;
                if(dist < 400 && inliers <= 30)
                    if(inliers >= size(best, 1))
                        best = temp;
                    end
                end
            end
            start = size(best,1);
            last = 1;
            for i = 1:(size(best, 1)-1)
                if(abs(best(i+1, 3) - best(i,3)) >= pi/2)
                    start = i;
                    last = i+1;
                    break;
                end
            end
            p_left = best(start, :);
            p_right = best(last, :);
            x_left = p_left(1);
            y_left = p_left(2);
            x_right = p_right(1);
            y_right = p_right(2);
            x_avg = (x_left + x_right) / 2;
            y_avg = (y_left + y_right) / 2;
            th = atan2(x_right - x_left,-y_right + y_left);
            pt = [x_avg, y_avg, th];
        end        
      end
end