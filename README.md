# Line Follower Simulator — Simulador com PID + Otimização por Algoritmo Genético

Simulador em **Pygame** de um robô seguidor de linha com controle **PID** e um script de otimização automática dos ganhos `Kp`, `Ki` e `Kd` usando **Algoritmo Genético** via [pymoo](https://pymoo.org/).  
O objetivo da otimização é encontrar parâmetros que permitam **completar a pista no menor tempo possível**, aplicando penalizações quando o robô sai da pista ou não termina no tempo limite.

---

## Estrutura do Repositório

```
.
├── line-follower.py      # Simulador principal em Pygame
├── galf.py               # Otimização de Kp, Ki, Kd com Algoritmo Genético
├── circuit_1.png         # Pista usada no simulador
├── basecode.cpp          # Código de referência (C++)
└── README.md             # Este arquivo
```

---

## Requisitos

- Python 3.3 ou superior
- Pymoo
- Matplotlib
- NumPy
- PyGame
- Dependências:
```bash
pip install pygame numpy matplotlib pymoo
```

---

## Como Executar

### Simulador (modo visual)
```bash
python line-follower.py
```
- Cada indivíduo do GA representa um conjunto de parâmetros `(Kp, Ki, Kd)`.
- A função de avaliação executa o simulador e mede o tempo para completar a pista.
- O objetivo é minimizar o tempo total.

---
## Como Contribuir

1. Faça um fork do repositório.
2. Crie uma branch para sua feature ou correção:
   ```bash
   git checkout -b minha-feature
   ```
3. Commit suas alterações:

```bash
Copiar
Editar
git commit -m "Adiciona nova funcionalidade X"
```
4. Envie para o seu fork:

```bash
Copiar
Editar
git push origin minha-feature
```
5. Abra um Pull Request explicando claramente suas mudanças.
