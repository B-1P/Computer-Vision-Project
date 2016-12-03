function [xs, ys] = getarrow2D(vs)
    xs = [vs(1,[1 2 2]);
          vs(1,[2 3 4])];
    ys = [vs(2,[1 2 2]);
          vs(2,[2 3 4])];
end