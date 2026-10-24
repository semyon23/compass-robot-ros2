# Локализация на активной карте (AMCL).
string status                    # ok | uncertain | lost | none (стек навигации не запущен или позы ещё не было)
float32 covariance               # σ положения, м (корень из большей дисперсии x/y); 0 при none
