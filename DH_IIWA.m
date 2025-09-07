% dh_or_mdh_from_urdf_iiwa14.m
clc; clear; close all;

% === Caminho do URDF (ajuste se necessário) ===
urdfPath = '/home/fernando/Mestrado/FLYMOV/catkin_ws/src/iiwa_stack/iiwa_description/urdf/iiwa14.urdf';
assert(isfile(urdfPath), 'URDF não encontrado:\n%s', urdfPath);

% Importa robô (RST)
mdl = importrobot(urdfPath);
mdl.DataFormat = 'row';

% Mapeamento pai-filho das 7 juntas revolutas (do seu showdetails)
parentLinks = {'iiwa_link_0','iiwa_link_1','iiwa_link_2','iiwa_link_3', ...
               'iiwa_link_4','iiwa_link_5','iiwa_link_6'};
childLinks  = {'iiwa_link_1','iiwa_link_2','iiwa_link_3','iiwa_link_4', ...
               'iiwa_link_5','iiwa_link_6','iiwa_link_7'};
jointNames  = {'iiwa_joint_1','iiwa_joint_2','iiwa_joint_3','iiwa_joint_4', ...
               'iiwa_joint_5','iiwa_joint_6','iiwa_joint_7'};

% === 1) Pergunta os ângulos ao usuário (em graus) ===
q_deg = zeros(1,numel(jointNames));
fprintf('Entre o valor em GRAUS para cada junta (ordem padrão do IIWA):\n');
for i = 1:numel(jointNames)
    prompt = sprintf('  %s (graus): ', jointNames{i});
    val = input(prompt);
    if isempty(val) || ~isnumeric(val) || ~isscalar(val)
        error('Entrada inválida para %s.', jointNames{i});
    end
    q_deg(i) = val;
end
q_rad = deg2rad(q_deg);

% Configuração de juntas (usa os valores fornecidos)
q = q_rad;

% === 2) Escolha da convenção (DH ou MDH) ===
conv = lower(strtrim(input('Gerar tabela "DH" ou "MDH"? ', 's')));
if ~ismember(conv, {'dh','mdh'})
    error('Escolha inválida. Digite "DH" ou "MDH".');
end

% ===== Funções auxiliares =====
wrapPi = @(x) atan2(sin(x), cos(x));  % normaliza ângulo para [-pi,pi]

% --- DH clássico: T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
extractDH = @(T) deal( ...
    atan2(T(2,1), T(1,1)), ...   % theta_total
    T(3,4), ...                   % d
    hypot(T(1,4), T(2,4)), ...    % a >= 0
    atan2(T(3,2), T(3,3)) ...     % alpha
);

% --- MDH (modificado): T = Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
% Rotação total: R = Rx(alpha)*Rz(theta) = [ cθ  -sθ   0
%                                            cαsθ cαcθ -sα
%                                            sαsθ sαcθ  cα ]
% Translação: p = [a;0;0] + R*[0;0;d] => [ a; -d sinα; d cosα ]
extractMDH = @(T) deal( ...
    ... theta_total:
    atan2(-T(1,2), T(1,1)), ...
    ... d (projeção em R(:,3)):
    dot(T(1:3,4), T(1:3,3)), ...
    ... a (projeção em eixo x após Rx(alpha), que resulta em px):
    T(1,4), ...
    ... alpha a partir de R(2,3)=-sinα, R(3,3)=cosα:
    atan2(-T(2,3), T(3,3)) ...
);

% ===== 3) Varredura por junta e extração =====
nq = numel(jointNames);
theta0 = zeros(1,nq); d = zeros(1,nq); a = zeros(1,nq); alpha = zeros(1,nq);

for i = 1:nq
    % Transformação pai->filho na configuração fornecida ^{parent}T_{child}
    T_pi = getTransform(mdl, q, childLinks{i}, parentLinks{i});

    switch conv
        case 'dh'
            [theta_total, d_i, a_i, alpha_i] = extractDH(T_pi);
        case 'mdh'
            [theta_total, d_i, a_i, alpha_i] = extractMDH(T_pi);
    end

    % Converte theta_total em offset theta0 removendo o valor de junta q(i)
    th0_i = wrapPi(theta_total - q(i));
    al_i  = wrapPi(alpha_i);

    theta0(i) = th0_i; d(i) = d_i; a(i) = a_i; alpha(i) = al_i;
end

% ===== 4) Tabela(s) =====
if strcmp(conv,'dh')
    fprintf('\nConvenção DH (clássica) assumida: T = Rz(theta)*Tz(d)*Tx(a)*Rx(alpha), com theta(q)=theta0+q.\n');
    Tfull = table( ...
        jointNames(:), ...
        theta0(:),            rad2deg(theta0(:)), ...
        d(:), a(:), ...
        alpha(:),             rad2deg(alpha(:)), ...
        'VariableNames', {'Joint','theta0_rad','theta0_deg','d_m','a_m','alpha_rad','alpha_deg'} ...
    );
    disp(Tfull);

    Tcompact = table( ...
        jointNames(:), ...
        rad2deg(theta0(:)), d(:), a(:), rad2deg(alpha(:)), ...
        'VariableNames', {'Joint','theta0_deg','d_m','a_m','alpha_deg'} ...
    );
    fprintf('\nTabela DH (compacta, graus):\n');
    disp(Tcompact);

    % Estrutura no workspace
    DH.theta0 = theta0; DH.d = d; DH.a = a; DH.alpha = alpha; DH.joints = jointNames;
    assignin('base','DH',DH);

else % 'mdh'
    fprintf('\nConvenção MDH (modificada) assumida: T = Rx(alpha)*Tx(a)*Rz(theta)*Tz(d), com theta(q)=theta0+q.\n');
    Tfull = table( ...
        jointNames(:), ...
        theta0(:),            rad2deg(theta0(:)), ...
        d(:), a(:), ...
        alpha(:),             rad2deg(alpha(:)), ...
        'VariableNames', {'Joint','theta0_rad','theta0_deg','d_m','a_m','alpha_rad','alpha_deg'} ...
    );
    disp(Tfull);

    Tcompact = table( ...
        jointNames(:), ...
        rad2deg(theta0(:)), d(:), a(:), rad2deg(alpha(:)), ...
        'VariableNames', {'Joint','theta0_deg','d_m','a_m','alpha_deg'} ...
    );
    fprintf('\nTabela MDH (compacta, graus):\n');
    disp(Tcompact);

    % Estrutura no workspace
    MDH.theta0 = theta0; MDH.d = d; MDH.a = a; MDH.alpha = alpha; MDH.joints = jointNames;
    assignin('base','MDH',MDH);
end

% ===== 5) Transformação homogênea 4x4 (base->flange e flange->base) =====
% **ADICIONADO**: monta ^baseT_flange a partir da tabela DH/MDH e do vetor q
T_base_flange = eye(4);
for i = 1:nq
    theta_i = theta0(i) + q(i);   % junta revoluta: theta = theta0 + q
    if strcmp(conv,'dh')
        % DH clássico: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
        A_i = rz(theta_i) * tz(d(i)) * tx(a(i)) * rx(alpha(i));
    else
        % MDH: Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
        A_i = rx(alpha(i)) * tx(a(i)) * rz(theta_i) * tz(d(i));
    end
    T_base_flange = T_base_flange * A_i;
end

% Inversa homogênea: ^flangeT_base
R = T_base_flange(1:3,1:3);
t = T_base_flange(1:3,4);
T_flange_base = eye(4);
T_flange_base(1:3,1:3) = R.';
T_flange_base(1:3,4)   = -R.'*t;

fprintf('\nTransformação homogênea 4x4 ^baseT_flange (%s):\n', upper(conv));
disp(T_base_flange);
fprintf('Transformação homogênea 4x4 ^flangeT_base (%s):\n', upper(conv));
disp(T_flange_base);

% Exporta para o workspace base (útil p/ usar depois)
assignin('base','T_base_flange',T_base_flange);
assignin('base','T_flange_base',T_flange_base);

% ===== Funções locais utilitárias (ADICIONADO) =====
function T = rx(alpha)
c = cos(alpha); s = sin(alpha);
T = [1 0  0 0;
     0 c -s 0;
     0 s  c 0;
     0 0  0 1];
end
function T = rz(theta)
c = cos(theta); s = sin(theta);
T = [ c -s 0 0;
      s  c 0 0;
      0  0 1 0;
      0  0 0 1 ];
end
function T = tx(a)
T = eye(4); T(1,4) = a;
end
function T = tz(d)
T = eye(4); T(3,4) = d;
end
