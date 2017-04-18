function [x, status, iA, lambda] = myMPC(Linv, f, Ain, bin, Aeq, beq, iA0)

assert(isa(Linv,'single'));
assert(isa(f,'single'));
assert(isa(Ain,'single'));
assert(isa(bin,'single'));
assert(isa(Aeq,'single'));
assert(isa(beq,'single'));
assert(isa(iA0,'logical'));

opt = mpcqpsolverOptions('single');
opt.IntegrityChecks = false;

[x, status, iA, lambda] = mpcqpsolver(Linv, f, Ain, bin, Aeq, beq, iA0, opt);

end

