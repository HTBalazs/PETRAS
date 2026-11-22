classdef MovingBuffer < handle
    properties (Access = private)
        WindowSize = 3;  % Alapértelmezett méret, ha a felhasználó nem ad meg semmit
        Data = [];       % [N sor (dimenziók) x K oszlop (időlépések)]
        Time = [];       % [1 x K] sorvektor az időpontoknak
        IsInitialized = false;
    end
    
    methods
        % Konstruktor: itt adhatod meg az ablak méretét (pl. b = MovingBuffer(5);)
        function obj = MovingBuffer(windowSize)
            if nargin > 0 && isnumeric(windowSize) && windowSize >= 2
                obj.WindowSize = windowSize;
            else
                obj.WindowSize = 3; % Biztonsági minimum a deriváláshoz
            end
            obj.IsInitialized = false;
        end
        
        % Új adat hozzáadása
        function push(obj, t_new, y_new)
            y_new = y_new(:); % Oszlopvektorrá kényszerítés
            K = obj.WindowSize;
            
            if ~obj.IsInitialized
                % Első alkalommal feltöltjük mind a K oszlopot az új adattal
                obj.Data = repmat(y_new, 1, K);
                
                % Az időpontokat fiktív, egymástól 1 másodpercre lévő múltbéli pontokként inicializáljuk
                obj.Time = t_new - (K-1:-1:0);
                obj.IsInitialized = true;
            else
                % Eltolás balra: az 1. oszlop (legrégebbi) kiesik, az új a végére megy
                obj.Data = [obj.Data(:, 2:end), y_new];
                obj.Time = [obj.Time(2:end), t_new];
            end
        end
        
        % Adatok lekérése
        function [t, y] = getValues(obj)
            t = obj.Time;
            y = obj.Data;
        end
        
        % Zajszűrt derivált számítása tetszőleges K pontra (Lineáris regresszió)
        function dydt = getFilteredDerivative(obj)
            t = obj.Time;
            y = obj.Data;
            K = obj.WindowSize;
            
            % Numerikus stabilitás: t1 kivonása
            t_rel = t - t(1);
            
            % Általános legkisebb négyzetek módszere K pontra
            sum_t   = sum(t_rel);
            sum_t2  = sum(t_rel.^2);
            sum_y   = sum(y, 2);
            sum_ty  = sum(y .* t_rel, 2);
            
            % A lineáris regresszió általánosított egyenlete:
            Sz = K * sum_ty - sum_t * sum_y;
            N  = K * sum_t2 - sum_t^2;
            
            if N == 0
                dydt = zeros(size(y, 1), 1);
            else
                dydt = Sz / N; % Visszaadja a dimenziónkénti deriváltakat [N x 1]
            end
        end
    end
end