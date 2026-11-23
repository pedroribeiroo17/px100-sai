# PX100: Cinemática e Geração de Movimento

Componente de **cinemática e geração de movimento** do projeto de Sistemas Autónomos Inteligentes (SAI, Universidade do Minho) para o manipulador **PincherX-100 (PX100)**, simulado em **CoppeliaSim** e controlado a partir de **MATLAB**.

O braço desloca-se até ao copo vermelho, fecha a garra e sobe. As coordenadas do copo **não são definidas à mão**: vêm do módulo de visão (`vision/`), que as devolve no referencial da base `/px100`. Esta documentação cobre apenas a parte de movimento. O relatório completo está em [`Relatorio_Cinematica_SAI.pdf`](Relatorio_Cinematica_SAI.pdf).

---

## Índice

1. [O manipulador](#1-o-manipulador)
2. [Cinemática direta](#2-cinemática-direta)
3. [Cinemática inversa](#3-cinemática-inversa)
4. [Geração de movimento e interpolações](#4-geração-de-movimento-e-interpolações)
5. [Ligação com a visão](#5-ligação-com-a-visão)
6. [Sequência de execução](#6-sequência-de-execução)
7. [Parâmetros](#7-parâmetros-getconfigm)
8. [Estrutura dos ficheiros](#8-estrutura-dos-ficheiros)
9. [Como executar](#9-como-executar)
10. [Resultados](#10-resultados)
11. [Limitações e trabalho futuro](#11-limitações-e-trabalho-futuro)

---

## 1. O manipulador

O PX100 tem 4 juntas rotativas (`waist`, `shoulder`, `elbow`, `wrist_angle`) e uma garra de dois dedos.

- A junta **waist** (θ₀) roda o braço em torno do eixo vertical.
- As juntas **shoulder, elbow e wrist_angle** (θ₁, θ₂, θ₃) têm eixos horizontais e paralelos. Para um dado θ₀, o braço move-se num plano vertical e comporta-se como um braço planar **RRR**.

| Símbolo | Valor | Descrição |
|---|---|---|
| `H` | 0,0931 m | altura do ombro |
| `L2` | 0,100 m | componente vertical do elo ombro–cotovelo |
| `Lm` | 0,035 m | desvio lateral do elo ombro–cotovelo |
| `LB` | 0,100 m | antebraço (cotovelo–pulso) |
| `LC` | 0,100 m | pulso → centro dos dedos (`wristToFingerCenter`) |

O desvio lateral faz com que o primeiro elo equivalha a um elo de comprimento

$$L_A = \sqrt{L_2^2 + L_m^2} \approx 0{,}106\ \text{m}, \qquad \beta = \operatorname{atan2}(L_m, L_2) \approx 19{,}3^\circ, \qquad \psi = \tfrac{\pi}{2} - \beta$$

inclinado de β em relação à vertical.

**Limites das juntas** (definidos em `arm_interface.m`):

| Junta | Mínimo | Máximo |
|---|---|---|
| waist | −180° | 180° |
| shoulder | −111° | 107° |
| elbow | −121° | 92° |
| wrist_angle | −100° | 123° |

---

## 2. Cinemática direta

Ficheiro: [`kinematics/directKinematicsPX100.m`](Simulador%20px100/kinematics/directKinematicsPX100.m)

Dado `q = [θ₀ θ₁ θ₂ θ₃]` (rad), devolve a posição do **pulso** (wrist) no referencial da base. θ₁ é medido a partir da vertical e θ₁+θ₂ é o ângulo do antebraço com a horizontal. No plano do braço:

$$r = L_A \sin(\theta_1 + \beta) + L_B \cos(\theta_1 + \theta_2)$$
$$h = H + L_A \cos(\theta_1 + \beta) - L_B \sin(\theta_1 + \theta_2)$$

Rodando o plano de θ₀ em torno de z:

$$\mathbf{p}_w = [\,r\cos\theta_0,\; r\sin\theta_0,\; h\,]$$

### Do pulso ao centro dos dedos

A cinemática direta e a inversa trabalham **só até ao pulso**. O elo da garra (`LC`) é somado fora delas, em `fingerCenterFK` (`pickRedCupPX100.m`). Com α₃ = θ₁+θ₂+θ₃ (inclinação da garra):

$$\mathbf{d} = [\cos\alpha_3\cos\theta_0,\; \cos\alpha_3\sin\theta_0,\; -\sin\alpha_3], \qquad \mathbf{p}_e = \mathbf{p}_w + L_C\,\mathbf{d}$$

Esta separação mantém as funções de cinemática independentes da ferramenta montada.

---

## 3. Cinemática inversa

Ficheiro: [`kinematics/inverseKinematicsPX100.m`](Simulador%20px100/kinematics/inverseKinematicsPX100.m)

```matlab
[joints, info] = inverseKinematicsPX100(targetWristBase, links, minJointPos, maxJointPos, ...
                                        alpha3, shoulderHeight, elbowConfiguration)
```

A solução é **analítica** (geométrica).

### Redundância

Há 4 incógnitas (θ₀…θ₃) e só 3 equações de posição, pelo que existem infinitas soluções, uma para cada inclinação da garra. A redundância é resolvida **impondo α₃** como parâmetro (nos testes, `α₃ = 0`, garra horizontal):

$$\theta_3 = \alpha_3 - \theta_1 - \theta_2$$

### Passos

1. **Waist:** $\theta_0 = \operatorname{atan2}(y_w, x_w)$
2. Redução a um braço planar de 2 elos (`LA` e `LB`): $X_w = \sqrt{x_w^2 + y_w^2}$, $Y_w = z_w - H$, $R = \sqrt{X_w^2 + Y_w^2}$, $\gamma = \operatorname{atan2}(Y_w, X_w)$
3. **Lei dos cossenos** no triângulo ombro–cotovelo–pulso:

$$\beta_1 = \arccos\frac{L_A^2 + R^2 - L_B^2}{2 L_A R}, \qquad \beta_2 = \arccos\frac{L_A^2 + L_B^2 - R^2}{2 L_A L_B}$$

4. **Configuração do cotovelo:**

| | θ₁ | θ₂ |
|---|---|---|
| `'up'` (por omissão) | $\frac{\pi}{2} - \beta - \beta_1 - \gamma$ | $\pi - \psi - \beta_2$ |
| `'down'` | $\frac{\pi}{2} - (\gamma - \beta_1 + \beta)$ | $-\pi + \beta_2 - \psi$ |

### Condições de existência

A função dá **erro** se:

- o pulso estiver fora da coroa esférica $|L_A - L_B| \le R \le L_A + L_B$ (≈ 6 mm a 206 mm do ombro);
- alguma junta exceder os seus limites mecânicos.

Devolve também `info.positionError`, o erro entre o alvo e a cinemática direta da solução obtida (verificação interna).

---

## 4. Geração de movimento e interpolações

Ficheiro: [`kinematics/pickRedCupPX100.m`](Simulador%20px100/kinematics/pickRedCupPX100.m)

O movimento tem duas fases, com **dois tipos de interpolação** diferentes, escolhidos de propósito.

### 4.1 Aproximação: interpolação linear no espaço das juntas

Função `moveJointsLinear`. Vai da pose atual até ao **ponto de aproximação** (80 mm acima do copo):

$$q(t) = q_{start} + t\,(q_{end} - q_{start}), \qquad t \in [0, 1]$$

- Discretizada em `approachSegments = 30` pontos, cada um mantido `stepsPerApproachPoint = 2` passos de simulação.
- É uma transição fluida e simples. Como não é uma reta no espaço cartesiano, serve apenas para chegar ao ponto elevado, onde a trajetória exata não importa.
- No fim, `waitUntilReached` confirma a chegada pela cinemática direta.

### 4.2 Descida e subida: interpolação linear cartesiana com perfil de mínimo jerk

Função `minJerkPath`. Do ponto de aproximação até ao ponto de pega (centro dos dedos):

$$\mathbf{p}_d(\tau) = \mathbf{p}_0 + s(\tau)\,(\mathbf{p}_f - \mathbf{p}_0), \qquad s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5, \qquad \tau = \frac{t}{T} \in [0, 1]$$

- O polinómio de 5.º grau dá **velocidade e aceleração nulas** no início e no fim. O braço pára sobre o copo sem solavancos.
- A trajetória é uma **reta vertical rigorosa** em Z, o que evita bater no copo.
- Discretizada em `descentSegments = 24` pontos, cada um executado com `stepsPerDescentPoint = 4` passos. Como os passos por ponto são constantes, os pontos equivalem a amostras uniformes no tempo.
- A inclinação da garra mantém-se constante (θ₁+θ₂+θ₃ = α₃).

### 4.3 Da trajetória cartesiana às juntas

Para cada ponto do percurso (`solveJointPath`):

1. calcula-se o waist a partir de (x, y) do centro dos dedos;
2. subtrai-se `LC` ao longo do eixo da garra para obter o **alvo do pulso**;
3. resolve-se a cinemática inversa;
4. rejeita-se o ponto se `positionError > maxIkError` (1e-5 m).

Se **algum** ponto for inalcançável, a função dá erro **antes** de o braço se mexer. A trajetória é validada na totalidade antes de ser executada.

### 4.4 Verificação durante o movimento

Em cada ponto, `waitUntilReached` lê as juntas reais (`get_joints`), calcula o centro dos dedos por cinemática direta (`fingerCenterFK`) e compara com o alvo. Continua a simular até o erro ficar abaixo de `positionTolerance` (10 mm) ou dá erro por timeout (`maxWaitSteps`).

No ponto de pega, `printGraspValidation` mostra as juntas lidas, a posição do pulso e do centro dos dedos, e o **erro final de posicionamento**:

$$e = \lVert \mathbf{p}_{alvo} - \mathbf{p}_{atingido} \rVert$$

### Resumo das interpolações

| Fase | Espaço | Perfil | Função | Segmentos |
|---|---|---|---|---|
| Aproximação (pose atual → 80 mm acima do copo) | Juntas | Linear | `moveJointsLinear` | 30 |
| Descida (ponto elevado → copo) | Cartesiano (reta em Z) | Mínimo jerk (5.º grau) | `minJerkPath` | 24 |
| Subida (copo → ponto elevado) | Cartesiano (reta em Z) | Mesmo percurso, em sentido inverso | `followPath` (com `flipud`) | 24 |

---

## 5. Ligação com a visão

O alvo do movimento **vem do módulo de visão**, não é uma constante no código. Em `main_program.m`:

```matlab
objectInfo = runVisionCupsPX100(sim_interface, true, cfg.cupHeight);
cup = findRedCup(objectInfo);          % escolhe o copo vermelho. para outra cor mudar no fim da main 'red'
pickRedCupPX100(robot, cfg, cup);      % usa cup.graspTargetBase
```

- `cup.graspTargetBase` é o ponto de pega **[X, Y, Z] em metros no referencial da base `/px100`**.
- X e Y são o centro do rebordo preto do copo, detetado na imagem e projetado para o plano da mesa com a calibração da câmara.
- Z é metade da altura conhecida do copo (`cfg.cupHeight`).

Os detalhes da deteção e da calibração estão em [`vision/README.md`](Simulador%20px100/vision/README.md).

---

## 6. Sequência de execução

1. Ligação ao CoppeliaSim e inicialização do braço com todas as juntas a 0°.
2. Abertura da garra.
3. A visão deteta os copos e devolve a posição do copo vermelho.
4. Definição do ponto de aproximação: `grasp + [0 0 approachClearance]`.
5. Geração do percurso de descida (`minJerkPath`) e cálculo das juntas de todos os pontos (`solveJointPath`).
6. **Aproximação** por interpolação linear nas juntas e confirmação da chegada.
7. **Descida** ao longo do percurso, com confirmação por cinemática direta em cada ponto.
8. Fecho da garra e espera (`stepsGripper`).
9. **Subida** pelo mesmo percurso, em sentido inverso.

---

## 7. Parâmetros (`getConfig.m`)

| Parâmetro | Valor | Significado |
|---|---|---|
| `cupHeight` | 0,070 m | altura do copo |
| `wristToFingerCenter` | 0,100 m | `LC`, pulso → centro dos dedos |
| `toolPitch` | 0° | α₃, garra horizontal |
| `elbow` | `'up'` | configuração do cotovelo |
| `approachClearance` | 0,080 m | altura do ponto de aproximação acima do alvo |
| `descentSegments` | 24 | pontos da descida/subida |
| `approachSegments` | 30 | pontos da aproximação articular |
| `positionTolerance` | 0,010 m | tolerância da confirmação por FK |
| `maxIkError` | 1e-5 m | erro máximo aceite na IK |
| `maxWaitSteps` | 500 | passos máximos à espera de cada ponto |
| `stepsPerApproachPoint` | 2 | passos de simulação por ponto da aproximação |
| `stepsPerDescentPoint` | 4 | passos de simulação por ponto da descida |
| `stepsGripper` | 60 | passos para abrir/fechar a garra |

---

## 8. Estrutura dos ficheiros

```
Simulador px100/
├── main_program.m                 # programa principal
├── getConfig.m                    # parâmetros
├── simulator_interface.m          # interface com o CoppeliaSim
├── Cenario_px100_vision.ttt       # cenário CoppeliaSim
├── kinematics/
│   ├── arm_interface.m            # interface com o braço (juntas, garra, limites, elos)
│   ├── directKinematicsPX100.m    # cinemática direta (pulso)
│   ├── inverseKinematicsPX100.m   # cinemática inversa analítica (pulso)
│   └── pickRedCupPX100.m          # trajetória, interpolações e execução da pega
├── vision/                        # deteção dos copos e calibração da câmara
├── outputs/kinematics_15_positions/   # resultados dos testes (Excel)
└── RemoteAPIClient.m, RemoteAPIObject.m, cbor.m, jeromq-0.5.2.jar   # ligação ao CoppeliaSim
```

---

## 9. Como executar

**Requisitos:** MATLAB e CoppeliaSim (com ZeroMQ Remote API).

1. Abrir `Cenario_px100_vision.ttt` no CoppeliaSim. **Não iniciar a simulação**, o MATLAB trata disso.
2. No MATLAB, na pasta `Simulador px100`:

```matlab
main_program
```

O programa adiciona automaticamente `vision/` e `kinematics/` ao path.

---

## 10. Resultados

Nos testes em simulação (ver `outputs/` e o relatório):

- Em **20 alvos aleatórios**, o erro de posicionamento final ficou entre **1,3 mm e 4,8 mm**.
- Nas pegas do copo, os erros foram de **0,27 mm, 0,11 mm e 0,08 mm**.
- A tolerância de 10 mm no seguimento da trajetória foi sempre cumprida.

---

## Autores

Grupo 5, Sistemas Autónomos Inteligentes, Universidade do Minho:
Cristiano Peixoto (PG63871), Fernando Novais (PG63878), Pedro Ribeiro (PG63902), Tiago Araújo (PG63912).
Docente: Prof.ª Estela Bicho Erlhagen.

Código base de interface com o simulador: Luís Louro e Estela Bicho.
