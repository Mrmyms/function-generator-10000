#ifndef MATH_EXPR_H
#define MATH_EXPR_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <Arduino.h>

#define BC_MAX_SIZE 128
#define OP_STACK_SIZE 32

enum BytecodeOp : uint8_t {
  BC_END = 0,
  BC_X,
  BC_T,
  BC_CONST,
  BC_ADD,
  BC_SUB,
  BC_MUL,
  BC_DIV,
  BC_POW,
  BC_NEG,
  BC_SIN,
  BC_COS,
  BC_TAN,
  BC_ASIN,
  BC_ACOS,
  BC_ATAN,
  BC_EXP,
  BC_LOG,
  BC_SQRT,
  BC_ABS,
  BC_SINC,
  BC_SQR,
  BC_FLOOR,
  BC_CEIL
};

struct MathExpression {
  char    exprStr[96];
  uint8_t bytecode[BC_MAX_SIZE];
  uint8_t bcLen;
  float   normScale;
  bool    isValid;
};

// Evaluate compiled bytecode at normalized x in [0.0, 1.0) and t = 2*PI*x in [0.0, 2*PI)
static inline float evalMathBytecode(const uint8_t *bc, float x, float t) {
  float stack[32];
  uint8_t sp = 0;
  const uint8_t *ip = bc;

  while (*ip != BC_END) {
    uint8_t op = *ip++;
    switch (op) {
      case BC_X:
        if (sp < 32) stack[sp++] = x;
        break;

      case BC_T:
        if (sp < 32) stack[sp++] = t;
        break;

      case BC_CONST: {
        float val;
        memcpy(&val, ip, sizeof(float));
        ip += sizeof(float);
        if (sp < 32) stack[sp++] = val;
        break;
      }

      case BC_ADD:
        if (sp >= 2) { float a = stack[--sp]; stack[sp - 1] += a; }
        break;

      case BC_SUB:
        if (sp >= 2) { float a = stack[--sp]; stack[sp - 1] -= a; }
        break;

      case BC_MUL:
        if (sp >= 2) { float a = stack[--sp]; stack[sp - 1] *= a; }
        break;

      case BC_DIV:
        if (sp >= 2) {
          float a = stack[--sp];
          stack[sp - 1] = (fabsf(a) < 1e-7f) ? 0.0f : stack[sp - 1] / a;
        }
        break;

      case BC_POW:
        if (sp >= 2) {
          float a = stack[--sp];
          float b = stack[sp - 1];
          stack[sp - 1] = (b < 0.0f && fabsf(a - floorf(a)) > 1e-5f) ? 0.0f : powf(b, a);
        }
        break;

      case BC_NEG:
        if (sp >= 1) stack[sp - 1] = -stack[sp - 1];
        break;

      case BC_SIN:
        if (sp >= 1) stack[sp - 1] = sinf(stack[sp - 1]);
        break;

      case BC_COS:
        if (sp >= 1) stack[sp - 1] = cosf(stack[sp - 1]);
        break;

      case BC_TAN:
        if (sp >= 1) stack[sp - 1] = tanf(stack[sp - 1]);
        break;

      case BC_ASIN:
        if (sp >= 1) {
          float v = constrain(stack[sp - 1], -1.0f, 1.0f);
          stack[sp - 1] = asinf(v);
        }
        break;

      case BC_ACOS:
        if (sp >= 1) {
          float v = constrain(stack[sp - 1], -1.0f, 1.0f);
          stack[sp - 1] = acosf(v);
        }
        break;

      case BC_ATAN:
        if (sp >= 1) stack[sp - 1] = atanf(stack[sp - 1]);
        break;

      case BC_EXP:
        if (sp >= 1) {
          float v = constrain(stack[sp - 1], -20.0f, 20.0f);
          stack[sp - 1] = expf(v);
        }
        break;

      case BC_LOG:
        if (sp >= 1) {
          float v = stack[sp - 1];
          stack[sp - 1] = (v <= 1e-7f) ? -16.0f : logf(v);
        }
        break;

      case BC_SQRT:
        if (sp >= 1) {
          float v = stack[sp - 1];
          stack[sp - 1] = (v <= 0.0f) ? 0.0f : sqrtf(v);
        }
        break;

      case BC_ABS:
        if (sp >= 1) stack[sp - 1] = fabsf(stack[sp - 1]);
        break;

      case BC_SINC:
        if (sp >= 1) {
          float v = stack[sp - 1];
          stack[sp - 1] = (fabsf(v) < 1e-5f) ? 1.0f : sinf(v) / v;
        }
        break;

      case BC_SQR:
        if (sp >= 1) stack[sp - 1] = stack[sp - 1] * stack[sp - 1];
        break;

      case BC_FLOOR:
        if (sp >= 1) stack[sp - 1] = floorf(stack[sp - 1]);
        break;

      case BC_CEIL:
        if (sp >= 1) stack[sp - 1] = ceilf(stack[sp - 1]);
        break;

      default:
        break;
    }
  }

  return (sp > 0) ? stack[0] : 0.0f;
}

// --- SHUNTING-YARD COMPILER ---
static inline int getPrecedence(uint8_t op) {
  switch (op) {
    case BC_ADD:
    case BC_SUB:  return 1;
    case BC_MUL:
    case BC_DIV:  return 2;
    case BC_NEG:  return 3;
    case BC_POW:  return 4;
    case BC_SIN:
    case BC_COS:
    case BC_TAN:
    case BC_ASIN:
    case BC_ACOS:
    case BC_ATAN:
    case BC_EXP:
    case BC_LOG:
    case BC_SQRT:
    case BC_ABS:
    case BC_SINC:
    case BC_SQR:
    case BC_FLOOR:
    case BC_CEIL: return 5;
    default:      return 0;
  }
}

static inline bool isRightAssociative(uint8_t op) {
  return (op == BC_POW || op == BC_NEG);
}

// Compile infix equation string into RPN bytecode
bool compileMathExpression(const char *expr, MathExpression *out) {
  if (expr == nullptr || strlen(expr) == 0 || out == nullptr) return false;

  strncpy(out->exprStr, expr, sizeof(out->exprStr) - 1);
  out->exprStr[sizeof(out->exprStr) - 1] = '\0';
  out->bcLen = 0;
  out->normScale = 1.0f;
  out->isValid = false;

  uint8_t opStack[OP_STACK_SIZE];
  int opTop = -1;

  const char *p = expr;
  bool expectUnary = true;

  while (*p) {
    // Skip whitespace
    if (isspace((unsigned char)*p)) {
      p++;
      continue;
    }

    // Handle implicit multiplication (e.g. 2x, 2sin(t), (x+1)(x-1))
    if (!expectUnary && (isdigit((unsigned char)*p) || *p == '.' || isalpha((unsigned char)*p) || *p == '(')) {
      uint8_t curOp = BC_MUL;
      int curPrec = getPrecedence(curOp);
      while (opTop >= 0 && opStack[opTop] != '(') {
        int topPrec = getPrecedence(opStack[opTop]);
        if (curPrec <= topPrec) {
          if (out->bcLen >= BC_MAX_SIZE) return false;
          out->bytecode[out->bcLen++] = opStack[opTop--];
        } else {
          break;
        }
      }
      if (opTop >= OP_STACK_SIZE - 1) return false;
      opStack[++opTop] = curOp;
      expectUnary = true;
    }

    // 1. Number literal
    if (isdigit((unsigned char)*p) || (*p == '.' && isdigit((unsigned char)*(p + 1)))) {
      char *endPtr;
      float val = strtof(p, &endPtr);
      if (endPtr == p) return false;
      p = endPtr;

      if (out->bcLen + 1 + sizeof(float) >= BC_MAX_SIZE) return false;
      out->bytecode[out->bcLen++] = BC_CONST;
      memcpy(&out->bytecode[out->bcLen], &val, sizeof(float));
      out->bcLen += sizeof(float);
      expectUnary = false;
      continue;
    }

    // 2. Identifier (variables, constants, functions)
    if (isalpha((unsigned char)*p)) {
      char id[16];
      int idLen = 0;
      while (isalnum((unsigned char)*p) || *p == '_') {
        if (idLen < 15) id[idLen++] = tolower((unsigned char)*p);
        p++;
      }
      id[idLen] = '\0';

      // Variables & Constants
      if (strcmp(id, "x") == 0) {
        if (out->bcLen >= BC_MAX_SIZE) return false;
        out->bytecode[out->bcLen++] = BC_X;
        expectUnary = false;
      } else if (strcmp(id, "t") == 0) {
        if (out->bcLen >= BC_MAX_SIZE) return false;
        out->bytecode[out->bcLen++] = BC_T;
        expectUnary = false;
      } else if (strcmp(id, "pi") == 0) {
        float val = 3.141592653589793f;
        if (out->bcLen + 1 + sizeof(float) >= BC_MAX_SIZE) return false;
        out->bytecode[out->bcLen++] = BC_CONST;
        memcpy(&out->bytecode[out->bcLen], &val, sizeof(float));
        out->bcLen += sizeof(float);
        expectUnary = false;
      } else if (strcmp(id, "e") == 0) {
        float val = 2.718281828459045f;
        if (out->bcLen + 1 + sizeof(float) >= BC_MAX_SIZE) return false;
        out->bytecode[out->bcLen++] = BC_CONST;
        memcpy(&out->bytecode[out->bcLen], &val, sizeof(float));
        out->bcLen += sizeof(float);
        expectUnary = false;
      }
      // Functions (push to operator stack)
      else {
        uint8_t funcOp = 0;
        if (strcmp(id, "sin") == 0)        funcOp = BC_SIN;
        else if (strcmp(id, "cos") == 0)   funcOp = BC_COS;
        else if (strcmp(id, "tan") == 0)   funcOp = BC_TAN;
        else if (strcmp(id, "asin") == 0)  funcOp = BC_ASIN;
        else if (strcmp(id, "acos") == 0)  funcOp = BC_ACOS;
        else if (strcmp(id, "atan") == 0)  funcOp = BC_ATAN;
        else if (strcmp(id, "exp") == 0)   funcOp = BC_EXP;
        else if (strcmp(id, "ln") == 0 || strcmp(id, "log") == 0) funcOp = BC_LOG;
        else if (strcmp(id, "sqrt") == 0)  funcOp = BC_SQRT;
        else if (strcmp(id, "abs") == 0)   funcOp = BC_ABS;
        else if (strcmp(id, "sinc") == 0)  funcOp = BC_SINC;
        else if (strcmp(id, "sqr") == 0)   funcOp = BC_SQR;
        else if (strcmp(id, "floor") == 0) funcOp = BC_FLOOR;
        else if (strcmp(id, "ceil") == 0)  funcOp = BC_CEIL;
        else return false; // Unknown identifier

        if (opTop >= OP_STACK_SIZE - 1) return false;
        opStack[++opTop] = funcOp;
        expectUnary = true;
      }
      continue;
    }

    // 3. Opening Parenthesis
    if (*p == '(') {
      if (opTop >= OP_STACK_SIZE - 1) return false;
      opStack[++opTop] = '(';
      expectUnary = true;
      p++;
      continue;
    }

    // 4. Closing Parenthesis
    if (*p == ')') {
      bool foundOpen = false;
      while (opTop >= 0) {
        if (opStack[opTop] == '(') {
          foundOpen = true;
          opTop--;
          break;
        }
        if (out->bcLen >= BC_MAX_SIZE) return false;
        out->bytecode[out->bcLen++] = opStack[opTop--];
      }
      if (!foundOpen) return false; // Mismatched parentheses

      // If function is on top of stack, pop it to output
      if (opTop >= 0 && getPrecedence(opStack[opTop]) == 5) {
        if (out->bcLen >= BC_MAX_SIZE) return false;
        out->bytecode[out->bcLen++] = opStack[opTop--];
      }

      expectUnary = false;
      p++;
      continue;
    }

    // 5. Operators
    uint8_t curOp = 0;
    if (*p == '+') curOp = BC_ADD;
    else if (*p == '-') {
      curOp = expectUnary ? BC_NEG : BC_SUB;
    }
    else if (*p == '*') curOp = BC_MUL;
    else if (*p == '/') curOp = BC_DIV;
    else if (*p == '^') curOp = BC_POW;
    else return false; // Invalid character

    int curPrec = getPrecedence(curOp);
    while (opTop >= 0 && opStack[opTop] != '(') {
      int topPrec = getPrecedence(opStack[opTop]);
      if ((isRightAssociative(curOp) && curPrec < topPrec) ||
          (!isRightAssociative(curOp) && curPrec <= topPrec)) {
        if (out->bcLen >= BC_MAX_SIZE) return false;
        out->bytecode[out->bcLen++] = opStack[opTop--];
      } else {
        break;
      }
    }

    if (opTop >= OP_STACK_SIZE - 1) return false;
    opStack[++opTop] = curOp;
    expectUnary = true;
    p++;
  }

  // Pop remaining operators
  while (opTop >= 0) {
    if (opStack[opTop] == '(' || opStack[opTop] == ')') return false; // Mismatched
    if (out->bcLen >= BC_MAX_SIZE) return false;
    out->bytecode[out->bcLen++] = opStack[opTop--];
  }

  if (out->bcLen >= BC_MAX_SIZE) return false;
  out->bytecode[out->bcLen] = BC_END;
  out->isValid = (out->bcLen > 0);

  if (out->isValid) {
    // Quick test scan across 128 points to determine peak amplitude and auto-scale if > 1.0
    float maxAbs = 0.0f;
    for (int step = 0; step < 128; step++) {
      float x = (float)step / 128.0f;
      float val = fabsf(evalMathBytecode(out->bytecode, x, x * 2.0f * (float)M_PI));
      if (!isnan(val) && !isinf(val) && val > maxAbs) {
        maxAbs = val;
      }
    }
    out->normScale = (maxAbs > 1.0f) ? (1.0f / maxAbs) : 1.0f;
  }

  return out->isValid;
}

#endif // MATH_EXPR_H
