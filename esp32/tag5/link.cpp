#include "link.h"

struct MyLink *init_link()
{
    struct MyLink *p;
    p = (struct MyLink *)malloc(sizeof(struct MyLink));
    p->next = NULL;
    return p;
}

void add_link(struct MyLink *p, uint16_t addr)
{
    struct MyLink *temp;
    temp = (struct MyLink *)malloc(sizeof(struct MyLink));
    temp->anchor_addr = addr;
    temp->next = p->next;
    p->next = temp;
}

struct MyLink *find_link(struct MyLink *p, uint16_t addr)
{
    while (p->next != NULL)
    {
        p = p->next;
        if (p->anchor_addr == addr)
            return p;
    }
    return NULL;
}

void fresh_link(struct MyLink *p, uint16_t addr, float range, float dbm, float temperature) // Updated function signature
{
    struct MyLink *temp;
    temp = find_link(p, addr);
    if (temp == NULL)
    {
        Serial.println("ERROR");
        return;
    }
    temp->range[2] = temp->range[1];
    temp->range[1] = temp->range[0];
    temp->range[0] = range;
    temp->dbm = dbm;
    temp->temperature = temperature; // Set temperature value
}

void print_link(struct MyLink *p)
{
    while (p->next != NULL)
    {
        p = p->next;
        Serial.print(p->anchor_addr, HEX);
        Serial.print(" - ");
        Serial.print(p->range[0]);
        Serial.print(" - ");
        Serial.print(p->dbm);
        Serial.print(" - ");
        Serial.println(p->temperature);
    }
}

void delete_link(struct MyLink *p, uint16_t addr)
{
    struct MyLink *temp;
    while (p->next != NULL)
    {
        if (p->next->anchor_addr == addr)
        {
            temp = p->next;
            p->next = temp->next;
            free(temp);
            return;
        }
        p = p->next;
    }
}

void make_link_json(struct MyLink *p, String *s)
{
    *s = "{";

    while (p->next != NULL)
    {
        p = p->next;
        String addr = String(p->anchor_addr);
        *s += "\"" + addr + "\": {";
        *s += "\"range\": " + String(p->range[0]) + ",";
        *s += "\"dbm\": " + String(p->dbm);
        //*s += "\"temperature\": " + String(p->temperature);
        *s += "}";

        if (p->next != NULL)
            *s += ",";
    }

    *s += "}";
}
